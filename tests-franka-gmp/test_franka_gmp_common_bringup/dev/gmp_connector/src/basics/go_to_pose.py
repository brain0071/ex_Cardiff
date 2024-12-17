import numpy as np

from adaptive_goal_region.robot_controller import RobotController


robot_controller = RobotController(real_robot=False, group_id="manipulator")

#pose = robot_controller.create_pose(np.array([-0.02283, 0.66093, 0.95682]), np.array([-3.10537, 0.11850, 2.49849]))

#robot_controller.go_to_pose_goal(pose)

robot_controller.go_to_capture_location()


