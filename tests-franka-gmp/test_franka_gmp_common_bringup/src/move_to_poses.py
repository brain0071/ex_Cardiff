#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Bool
import os
from gazebo_msgs.msg import ModelStates


class move_to_poses:

    def __init__(self):
        return


# Dynamically get the home directory
home_dir = os.path.expanduser("~")
# Construct the desired path relative to the home directory
pose_path = os.path.join(
    home_dir,
    "dev",
    "storage",
    "agr_output_single.txt",
)


def add_collision_objects(robot, scene, cube_pose, cyl_pose):
    # Add table
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = robot.get_planning_frame()
    table_pose.pose.position.x = 0.00
    table_pose.pose.position.y = 0.00
    table_pose.pose.position.z = 0.71
    table_pose.pose.orientation.x = 0.00
    table_pose.pose.orientation.y = 0.00
    table_pose.pose.orientation.z = 0.00
    table_pose.pose.orientation.w = 1.00
    table_name = "table"
    scene.add_box(table_name, table_pose, size=(0.8, 1.58, 0.019708))

    # Add top shelf
    top_shelf_pose = geometry_msgs.msg.PoseStamped()
    top_shelf_pose.header.frame_id = robot.get_planning_frame()
    top_shelf_pose.pose.position.x = 0.00
    top_shelf_pose.pose.position.y = 0.965
    top_shelf_pose.pose.position.z = 1.19
    top_shelf_pose.pose.orientation.x = 0.00
    top_shelf_pose.pose.orientation.y = 0.00
    top_shelf_pose.pose.orientation.z = 0.00
    top_shelf_pose.pose.orientation.w = 1.00
    top_shelf_name = "top_shelf"
    scene.add_box(top_shelf_name, top_shelf_pose, size=(0.88, 0.4, 0.02))

    # Add bottom shelf
    bot_shelf_pose = geometry_msgs.msg.PoseStamped()
    bot_shelf_pose.header.frame_id = robot.get_planning_frame()
    bot_shelf_pose.pose.position.x = 0.00
    bot_shelf_pose.pose.position.y = 0.965
    bot_shelf_pose.pose.position.z = 0.03
    bot_shelf_pose.pose.orientation.x = 0.00
    bot_shelf_pose.pose.orientation.y = 0.00
    bot_shelf_pose.pose.orientation.z = 0.00
    bot_shelf_pose.pose.orientation.w = 1.00
    bot_shelf_name = "bot_shelf"
    scene.add_box(bot_shelf_name, bot_shelf_pose, size=(0.88, 0.4, 0.06))

    # Add left shelf
    left_shelf_pose = geometry_msgs.msg.PoseStamped()
    left_shelf_pose.header.frame_id = robot.get_planning_frame()
    left_shelf_pose.pose.position.x = 0.45
    left_shelf_pose.pose.position.y = 0.965
    left_shelf_pose.pose.position.z = 0.6
    left_shelf_pose.pose.orientation.x = 0.00
    left_shelf_pose.pose.orientation.y = 0.00
    left_shelf_pose.pose.orientation.z = 0.00
    left_shelf_pose.pose.orientation.w = 1.00
    left_shelf_name = "left_shelf"
    scene.add_box(left_shelf_name, left_shelf_pose, size=(0.02, 0.4, 1.2))

    # Add right shelf
    right_shelf_pose = geometry_msgs.msg.PoseStamped()
    right_shelf_pose.header.frame_id = robot.get_planning_frame()
    right_shelf_pose.pose.position.x = -0.45
    right_shelf_pose.pose.position.y = 0.965
    right_shelf_pose.pose.position.z = 0.6
    right_shelf_pose.pose.orientation.x = 0.00
    right_shelf_pose.pose.orientation.y = 0.00
    right_shelf_pose.pose.orientation.z = 0.00
    right_shelf_pose.pose.orientation.w = 1.00
    right_shelf_name = "right_shelf"
    scene.add_box(right_shelf_name, right_shelf_pose, size=(0.02, 0.4, 1.2))

    # Add back shelf
    back_shelf_pose = geometry_msgs.msg.PoseStamped()
    back_shelf_pose.header.frame_id = robot.get_planning_frame()
    back_shelf_pose.pose.position.x = 0.00
    back_shelf_pose.pose.position.y = 1.165
    back_shelf_pose.pose.position.z = 0.6
    back_shelf_pose.pose.orientation.x = 0.00
    back_shelf_pose.pose.orientation.y = 0.00
    back_shelf_pose.pose.orientation.z = 0.00
    back_shelf_pose.pose.orientation.w = 1.00
    back_shelf_name = "back_shelf"
    scene.add_box(back_shelf_name, back_shelf_pose, size=(0.9, 0.01, 1.2))

    # Add low shelf
    low_shelf_pose = geometry_msgs.msg.PoseStamped()
    low_shelf_pose.header.frame_id = robot.get_planning_frame()
    low_shelf_pose.pose.position.x = 0.00
    low_shelf_pose.pose.position.y = 0.965
    low_shelf_pose.pose.position.z = 0.43
    low_shelf_pose.pose.orientation.x = 0.00
    low_shelf_pose.pose.orientation.y = 0.00
    low_shelf_pose.pose.orientation.z = 0.00
    low_shelf_pose.pose.orientation.w = 1.00
    low_shelf_name = "low_shelf"
    scene.add_box(low_shelf_name, low_shelf_pose, size=(0.88, 0.4, 0.02))

    # Add high shelf
    high_shelf_pose = geometry_msgs.msg.PoseStamped()
    high_shelf_pose.header.frame_id = robot.get_planning_frame()
    high_shelf_pose.pose.position.x = 0.00
    high_shelf_pose.pose.position.y = 0.965
    high_shelf_pose.pose.position.z = 0.8
    high_shelf_pose.pose.orientation.x = 0.00
    high_shelf_pose.pose.orientation.y = 0.00
    high_shelf_pose.pose.orientation.z = 0.00
    high_shelf_pose.pose.orientation.w = 1.00
    high_shelf_name = "high_shelf"
    scene.add_box(high_shelf_name, high_shelf_pose, size=(0.88, 0.4, 0.02))

    # Add cube
    cube_pose.header.frame_id = robot.get_planning_frame()
    cube_name = "box"
    scene.add_box(cube_name, cube_pose, size=(0.15, 0.15, 0.15))

    # Add cylinder
    cyl_pose.header.frame_id = robot.get_planning_frame()
    cyl_name = "cylinder"
    scene.add_cylinder(cyl_name, cyl_pose, height=0.23, radius=0.02)


def model_states_callback(msg):
    global cube_pose, cyl_pose

    try:
        cube_index = msg.name.index("box")
        cyl_index = msg.name.index("cylinder")

        cube_pose = geometry_msgs.msg.PoseStamped()
        cube_pose.pose = msg.pose[cube_index]

        cyl_pose = geometry_msgs.msg.PoseStamped()
        cyl_pose.pose = msg.pose[cyl_index]

    except ValueError as e:
        rospy.logerr(f"Error finding models: {e}")


def open_gripper():
    # ! OPEN GRIPPER
    move_group_gripper = moveit_commander.MoveGroupCommander("panda_hand")
    open_joint_positions = [0.04, 0.04]
    move_group_gripper.go(open_joint_positions, wait=True)
    move_group_gripper.stop()
    rospy.sleep(1)


if __name__ == "__main__":

    poses = []
    for line in open(pose_path):
        pose = [float(i) for i in line.split()]
        poses.append(pose)

    rospy.init_node("move_to_pose_node")

    moveit_commander.roscpp_initialize(sys.argv)

    # ! INITIALIZE MOVEIT CLASSES
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander("panda_arm")
    group.set_end_effector_link(group.get_end_effector_link())
    print(group.get_end_effector_link())
    group.set_planning_time(10)
    group.set_goal_tolerance(0.01)
    goal_reached = True

    global cube_pose, cyl_pose
    cube_pose = geometry_msgs.msg.PoseStamped()
    cyl_pose = geometry_msgs.msg.PoseStamped()

    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    rospy.sleep(2)  # Wait for the topic to be populated

    add_collision_objects(robot, scene, cube_pose, cyl_pose)

    # ! ATTEMPT EACH GRASP POSE UNTIL SUCCESSFUL
    for index in range(0, 200):

        rospy.loginfo(f"Pose: {index+1}")
        open_gripper()

        # ! DEFINE THE GOAL
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = poses[index][0]
        pose_goal.position.y = poses[index][1]
        pose_goal.position.z = poses[index][2]

        pose_goal.orientation.x = poses[index][3]
        pose_goal.orientation.y = poses[index][4]
        pose_goal.orientation.z = poses[index][5]
        pose_goal.orientation.w = poses[index][6]

        group.set_pose_target(pose_goal)

        plan_success, plan, planning_time, error_code = group.plan()
        if plan.joint_trajectory.points:
            rospy.loginfo("Successful.")
            execute = group.execute(plan, wait=True)
            sys.exit()
        else:
            rospy.loginfo("Unsuccessful.")

        group.stop()
        group.clear_pose_targets()
