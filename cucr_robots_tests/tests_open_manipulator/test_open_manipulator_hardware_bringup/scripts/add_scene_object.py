#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import time

if __name__ == "__main__":
    rospy.init_node("name_node")
    rospy.loginfo("Starting name_node.")

    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    time.sleep(2)

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = -0.005
    scene.add_box("table", p, (0.55, 0.55, 0.01))
