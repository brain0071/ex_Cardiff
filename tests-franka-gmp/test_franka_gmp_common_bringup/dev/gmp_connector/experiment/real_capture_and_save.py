import numpy as np
import time
import os


from robot_controller_real import ROSController


ros_controller = ROSController(real_robot=True)

HOME_DIR = os.environ["HOME"]
FILE_DIR = os.path.join(
    HOME_DIR, "dev", "gmp_connector", "storage", "undefined", "temp"
)

# Open the gripper
ros_controller.hand_open()
# Go to Image Capturing Location
ros_controller.go_to_capture_location()
# Save image, depth data and camera info
ros_controller.capture_image_and_save_info(FILE_DIR)

ros_controller.run_segmentation_and_grasp_net(
    os.path.join(FILE_DIR, "raw_capture.npy"), "http://127.0.0.1/run"
)
