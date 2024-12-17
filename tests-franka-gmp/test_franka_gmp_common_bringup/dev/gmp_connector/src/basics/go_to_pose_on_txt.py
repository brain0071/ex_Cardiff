import time

import numpy as np
from scipy.spatial.transform import Rotation as R
from adaptive_goal_region.robot_controller import RobotController
from tf.transformations import quaternion_from_euler


file_path = "storage/spline_finals/agr_output.txt"
file = open(file_path, "r+")
poses = []
for line in file.read().split("\n"):
    if not line:
        continue
    pose = []
    for i in line.split(" "):
        if i:
            pose.append(float(i))
    poses.append(np.array(pose[:6]))
    poses.append(np.array(pose[6:]))

robot_controller = RobotController(real_robot=False, group_id="manipulator")

i = 0
for pose in poses:
    pose = robot_controller.create_pose(pose[:3], pose[3:])
    robot_controller.go_to_pose_goal(pose)
    time.sleep(1)
    i += 1
    if i > 3:
        break

