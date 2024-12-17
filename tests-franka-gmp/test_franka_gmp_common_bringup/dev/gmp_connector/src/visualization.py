from adaptive_goal_region.visualization.visualization_interpolation import visualize_grasps
from adaptive_goal_region.robot_controller import RobotController

robot_controller = RobotController(real_robot=False)

poses = robot_controller.transform_grasping_poses()

visualize_grasps(None, poses)
