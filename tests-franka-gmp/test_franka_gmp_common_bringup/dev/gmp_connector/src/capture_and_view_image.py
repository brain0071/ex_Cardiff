from adaptive_goal_region.robot_controller import RobotController

robot_controller = RobotController(real_robot=False)
# Open the gripper
robot_controller.hand_open()
# Go to Image Capturing Location
robot_controller.go_to_capture_location()
# Save image, depth data and camera info
robot_controller.capture_image_and_save_info()
# View image
robot_controller.view_image()  # @todo There is an issue about this command!
