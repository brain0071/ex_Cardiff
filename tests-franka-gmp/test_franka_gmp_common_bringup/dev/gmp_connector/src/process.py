from adaptive_goal_region.robot_controller import RobotController
from adaptive_goal_region.src.agr_helper import save_agr_data_as_txt
from adaptive_goal_region.src.agr_main import agr
from adaptive_goal_region.visualization.visualization_interpolation import visualize_grasps

robot_controller = RobotController(real_robot=False)

robot_controller.hand_open()

# Go To Capture Location to Ensure Frame Transformation
robot_controller.go_to_capture_location()

matrices_data = robot_controller.read_grasping_poses_file()  # orientation as quaternion

poses = robot_controller.convert_matrices_to_array(matrices_data, sort=True)  # orientation as quaternion

poses = robot_controller.array_frame_transformation(poses)  # orientation as quaternion

poses = robot_controller.array_rotation(poses, 90)

adaptive_goal_region_data = agr(poses, rotate=False, return_euler=False)  # Includes z-axis 90 degree rotation

# Save Single Poses
save_agr_data_as_txt(poses, path="storage/spline_finals/agr_output_single.txt", unique=True)

# Save AGR Data
save_agr_data_as_txt(adaptive_goal_region_data, path="storage/spline_finals/agr_output_region.txt", unique=False)

# Remove Comments to Enable Visualization

visualize_grasps(adaptive_goal_region_data)
