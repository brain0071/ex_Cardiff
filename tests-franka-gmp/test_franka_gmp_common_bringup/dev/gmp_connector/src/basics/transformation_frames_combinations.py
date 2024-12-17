import time

import numpy as np

from adaptive_goal_region.visualization.visualization_interpolation import visualize_grasps
from adaptive_goal_region.robot_controller import RobotController


robot_controller = RobotController(real_robot=False, group_id='arm')

pose = pose_first = np.array([-0.00694, 0.75659, 1.11397, -2.46658, 0.16737, -0.94575])
robot_controller.create_object(robot_controller.create_pose(pose[:3], pose[3:]))

base_pos = np.array([-0.05158488, 0.7086437, 1.1878864, 0.82614, -0.45454, 0.080436, -0.32313])

def euclidean_distance_3d(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.sqrt(np.sum((point1 - point2) ** 2))


links = [
    "panda_EE",
    "panda_K",
    "panda_NE",
    "panda_hand",
    "panda_hand_sc",
    "panda_hand_tcp",
    "panda_link8",
    "world",
]
min_dist = 1000000.0
for link1 in links:
    for link2 in links:
        if link1 == link2:
            continue
        pointer = ""
        pose = robot_controller.frame_transformation(pose, link1, link2)
        pose = robot_controller.pose_to_array(pose)
        position = pose
        distance = euclidean_distance_3d(position, base_pos)
        if distance < 1:
            if distance < min_dist:
                min_dist = distance
                pointer = "*"

            print(f"{pointer} Distance = {distance} | {link1} - {link2}")
        pose = robot_controller.frame_transformation(pose, link2, link1)
        pose = robot_controller.pose_to_array(pose)
        position = pose
        distance = euclidean_distance_3d(position, base_pos)
        if distance < 1:
            if distance < min_dist:
                min_dist = distance
                pointer = "*"

            print(f"{pointer} Distance = {distance} | {link2} - {link1}")



#pose = robot_controller.frame_transformation(pose, "panda_hand_tcp", "panda_link8")
# pose_second = robot_controller.pose_to_array(pose)
# pose = robot_controller.frame_transformation(pose, "panda_link8", "world")
#print(pose_second[:3])
# pose_second = robot_controller.create_pose(pose_second)
#pose = robot_controller.frame_transformation(pose, "panda_link8", "world")



#robot_controller.go_to_pose_goal(robot_controller.create_pose(pose_first[:3], pose_first[3:]))
#time.sleep(5)
# robot_controller = RobotController(real_robot=False, group_id='arm')
#robot_controller.go_to_pose_goal(pose_second)



