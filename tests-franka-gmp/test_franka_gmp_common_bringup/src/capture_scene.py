#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from typing import Optional
from sensor_msgs.msg import CameraInfo, Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import threading
import os
import sys
import argparse

global save_path

# Initialize CvBridge
bridge = CvBridge()

# Function to handle shutdown
def shutdown_callback():
    print("Capture saved.")
    rospy.signal_shutdown("Timer expired")


class CaptureScene:

    def __init__(self):
        self.rgb_array = None
        self.depth_array = None
        self.camera_info = None

    def rgb_callback(self, data):
        self.rgb_array = bridge.imgmsg_to_cv2(data, "bgr8")
        rospy.sleep(1)

    def depth_callback(self, data):
        depth_img = bridge.imgmsg_to_cv2(data, "32FC1")
        self.depth_array = np.array(depth_img, dtype=np.float64) / 1000.0  # Convert to meters
        rospy.sleep(1)

    def camera_info_callback(self, data):
        cam_info = data.K
        self.camera_info = np.array(
            [
                [cam_info[0], 0.0, cam_info[2]],
                [0.0, cam_info[4], cam_info[5]],
                [0.0, 0.0, 1.0],
            ]
        )
        rospy.sleep(1)

    def filter_color(self, color_img: np.ndarray, depth_data: np.ndarray) -> np.ndarray:
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])
        lower_brown = np.array([10, 20, 20])
        upper_brown = np.array([52, 255, 255])
        hsv_image = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
        mask_brown = cv2.inRange(hsv_image, lower_brown, upper_brown)
        mask_combined = cv2.bitwise_or(mask_blue, mask_brown)
        mask_non_brown_blue = cv2.bitwise_not(mask_brown)
        non_red_blue_objects_hsv = cv2.bitwise_and(
            hsv_image, hsv_image, mask=mask_non_brown_blue
        )
        filtered_depth_data = cv2.bitwise_and(
            depth_data, depth_data, mask=mask_non_brown_blue
        )
        return filtered_depth_data

    def capture_image_and_save_info(self) -> str:
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw",
            Image,
            self.depth_callback,
        )
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info",
            CameraInfo,
            self.camera_info_callback,
        )
        rospy.sleep(5)

        if self.rgb_array is None or self.depth_array is None or self.camera_info is None:
            rospy.logerr("Failed to capture complete scene data.")
            return ""

        filtered_depth_array = self.filter_color(self.rgb_array, self.depth_array)
        data_dict = {
            "rgb": np.array(self.rgb_array),
            "depth": np.array(filtered_depth_array),
            "label": np.zeros((self.rgb_array.shape[0], self.rgb_array.shape[1]), dtype=np.uint8),
            "K": np.array(self.camera_info),
        }

        np.save(save_path, data_dict)
        rospy.loginfo(f"Scene data saved to {save_path}")
        return save_path


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="AGR - Gazebo Image Capture and Save Results"
    )

    parser.add_argument(
        "-o", "--output_dir", type=str, required=True, help="The output dir"
    )

    args = parser.parse_args()
    rospy.init_node("capture_scene_node")

    # Construct the path for saving scene data
    save_path = os.path.join(args.output_dir, "scene.npy")

    capture_scene_obj = CaptureScene()

    rospy.Subscriber("capture_reached", Bool, lambda msg: capture_scene_obj.capture_image_and_save_info())
    
    # Start a timer to shut down the node after 20 seconds
    timer = threading.Timer(20, shutdown_callback)
    timer.start()

    rospy.spin()
