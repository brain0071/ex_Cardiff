import numpy as np

from scipy.spatial.transform import Rotation as R
from adaptive_goal_region.robot_controller import RobotController


def rotate_quaternion(quaternion, axis='x', degrees=90):
    original_rotation = R.from_quat(quaternion)
    original_rotation = R.from_euler(axis, degrees, degrees=True) * original_rotation
    return original_rotation.as_quat()


robot_controller = RobotController(real_robot=False, group_id="manipulator")
robot_controller.go_to_capture_location()
pose_array = np.array([[0.03301899880170822, 0.9960200190544128, -0.08279299736022949, 0.11334100365638733], [-0.4533669948577881, 0.08875100314617157, 0.8868939876556396, -0.17246000468730927], [0.8907120227813721, 0.008251000195741653, 0.45449298620224, 1.001459002494812], [0.0, 0.0, 0.0, 1.0]])

pose_array = robot_controller.matrix_to_pose_quaternion(pose_array)
pose = robot_controller.transform_camera_to_world(pose_array)
print(pose)
pose_array = robot_controller.pose_to_array(pose)

axes = ['x', 'y', 'z']

robot_controller.hand_open()
print(pose_array[3:])
orientation = rotate_quaternion(pose_array[3:], 'z', -90)
print(orientation)
pose = robot_controller.create_pose(pose_array[:3], orientation)
# robot_controller.go_to_pose_goal(pose)

# Going with z 90 degree
# Going with z 90 degree and z 180 degrees

for axis in axes:
    continue
    orientation = rotate_quaternion(pose_array[3:], axis, 90)
    pose = robot_controller.create_pose(pose_array[:3], orientation)
    print(f" Going with {axis} 90 degrees")
    robot_controller.go_to_pose_goal(pose)
    for axis2 in axes:
        orientation2 = rotate_quaternion(orientation, axis2, 180)
        pose = robot_controller.create_pose(pose_array[:3], orientation2)
        print(f" Going with {axis} 90 degree and {axis2} 180 degrees")
        robot_controller.go_to_pose_goal(pose)

