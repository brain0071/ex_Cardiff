import numpy as np

from adaptive_goal_region.robot_controller import RobotController

robot_controller = RobotController(real_robot=False)

pose = robot_controller.create_pose(
    np.array([-0.00694, 0.75659, 1.11397,]),
    np.array([-2.46658, 0.16737, -0.94575])
)

robot_controller.create_object(pose)

robot_controller.go_to_capture_location()
