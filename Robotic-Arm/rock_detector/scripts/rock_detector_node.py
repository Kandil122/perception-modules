#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from ultralytics import YOLO

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError

class RockDetectorNode:
    """
    This ROS node detects rocks using a YOLOv8 model, calculates their
    3D position, and publishes this as a PoseStamped message.
    It also publishes a debug image with bounding boxes drawn.
    """
    def __init__(self):
        rospy.init_node('rock_detector_node')
        
        self.load_params()
        self.bridge = CvBridge()
        
        try:
            self.model = YOLO(self.model_path)
            rospy.loginfo("--- Successfully loaded YOLOv8 model! ---")
        except Exception as e:
            rospy.logfatal(f"--- Failed to load YOLOv8 model: {e} ---")
            rospy.signal_shutdown("Model loading failed.")
            return
            
        self.target_class_id = self.get_class_id(self.target_class_name)
        if self.target_class_id is None:
            rospy.logfatal(f"--- Target class '{self.target_class_name}' not found in model! ---")
            rospy.signal_shutdown("Invalid target class.")
            return

        self.pose_publisher = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        # DEBUG: Create a publisher for the visual debug image
        self.debug_image_publisher = rospy.Publisher("/rock_detector/debug_image", Image, queue_size=1)

        self.camera_intrinsics = None
        self.frame_count = 0 # DEBUG: To count processed frames

        image_sub = message_filters.Subscriber(self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
        self.time_synchronizer = message_filters.TimeSynchronizer([image_sub, depth_sub, info_sub], 10)
        self.time_synchronizer.registerCallback(self.camera_callback)
        rospy.loginfo("--- Rock detector node initialized. Waiting for camera data... ---")

    def load_params(self):
        """Loads parameters from the node's private namespace."""
        self.image_topic = rospy.get_param('~image_topic')
        self.depth_topic = rospy.get_param('~depth_topic')
        self.camera_info_topic = rospy.get_param('~camera_info_topic')
        self.model_path = rospy.get_param('~model_path')
        self.confidence_threshold = rospy.get_param('~confidence_threshold')
        self.target_class_name = rospy.get_param('~target_class_name')
        self.pose_topic = rospy.get_param('~pose_topic')
        rospy.loginfo("--- Parameters loaded. Confidence threshold: %.2f ---", self.confidence_threshold)

    def get_class_id(self, target_name):
        try:
            names = self.model.names
            for class_id, class_name in names.items():
                if class_name.lower() == target_name.lower():
                    rospy.loginfo("--- Found target class '%s' with ID: %d ---", target_name, class_id)
                    return int(class_id)
        except Exception as e:
            rospy.logerr(f"Could not get class names from model: {e}")
        return None

    def camera_callback(self, rgb_msg, depth_msg, info_msg):
        self.frame_count += 1
        rospy.loginfo_throttle(5, f"--- Processing frame #{self.frame_count} ---") # Log every 5 seconds

        if self.camera_intrinsics is None:
            self.camera_intrinsics = {'fx': info_msg.K[0], 'fy': info_msg.K[4], 'cx': info_msg.K[2], 'cy': info_msg.K[5]}
            self.camera_optical_frame_id = info_msg.header.frame_id
            rospy.loginfo(f"--- Camera intrinsics and frame_id ('%s') received. ---", self.camera_optical_frame_id)

        try:
            cv_image_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_image_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(f"CV-Bridge Error: {e}")
            return

        # Run Inference
        results = self.model.predict(source=cv_image_rgb, conf=self.confidence_threshold, verbose=False)
        detections = results[0].boxes.data

        # DEBUG: Log the number of raw detections found (before filtering for 'rock')
        if len(detections) == 0:
            rospy.loginfo_throttle(5, "--- YOLO found NO objects in this frame. ---")
        else:
            rospy.loginfo_throttle(2, f"--- YOLO found {len(detections)} raw object(s) in this frame. ---")

        found_target = False
        # DEBUG: Create a copy of the image to draw on
        debug_image = cv_image_rgb.copy()

        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            class_id = int(class_id)
            
            # DEBUG: Draw bounding box for EVERY detected object
            label = f"{self.model.names[class_id]} {conf:.2f}"
            color = (255, 0, 0) # Blue for non-target objects
            
            # Process the TARGET object
            if class_id == self.target_class_id:
                found_target = True
                color = (0, 255, 0) # Green for the target rock
                
                center_x, center_y = int((x1 + x2) / 2), int((y1 + y2) / 2)
                
                try:
                    depth_value = cv_image_depth[center_y, center_x]
                    if np.isnan(depth_value) or depth_value <= 0:
                        # DEBUG: Make this warning more visible
                        rospy.logwarn(f"--- Found rock, but depth is INVALID ({depth_value}) at pixel ({center_x}, {center_y}). Skipping. ---")
                        continue # Skip to the next detection
                        
                    depth_m = depth_value / 1000.0 if cv_image_depth.dtype == np.uint16 else depth_value
                    point_3d = self.deproject_pixel_to_point(center_x, center_y, depth_m)
                    self.publish_rock_pose(point_3d, rgb_msg.header)

                except IndexError:
                    rospy.logwarn("--- Centroid pixel is out of depth image bounds. ---")

            # DEBUG: Draw the rectangle and label on the debug image
            cv2.rectangle(debug_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(debug_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # DEBUG: Publish the debug image regardless of whether a rock was found
        try:
            debug_img_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_img_msg.header = rgb_msg.header # Give it the same timestamp and frame
            self.debug_image_publisher.publish(debug_img_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Could not publish debug image: {e}")

    def deproject_pixel_to_point(self, u, v, depth):
        x = (u - self.camera_intrinsics['cx']) * depth / self.camera_intrinsics['fx']
        y = (v - self.camera_intrinsics['cy']) * depth / self.camera_intrinsics['fy']
        return (x, y, depth)

    def publish_rock_pose(self, point_3d, header):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = header.stamp
        pose_msg.header.frame_id = self.camera_optical_frame_id
        pose_msg.pose.position = Point(x=point_3d[0], y=point_3d[1], z=point_3d[2])
        pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pose_publisher.publish(pose_msg)

if __name__ == '__main__':
    try:
        RockDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass