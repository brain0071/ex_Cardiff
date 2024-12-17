#include <iostream>
#include <vector>

#include <cmath>
#include <cstdlib>
#include <string>

// ROS stuff
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_state/robot_state.h>

#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/PointHeadGoal.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <actionlib/client/simple_action_client.h>

//!  TiagoMoveItDemo class.
/*!
 * Tiago MoveIt! Demo.
 *
 */
class Tiago
{
public:
    //! Constructor
    Tiago();
    //! Function for waiting for user confirmation
    void waitForUserConfirmation(const std::string &next_task);
    //! Run
    void run();

private:
    // ROS
    ros::NodeHandle node_handler_, local_nh_;
    bool replan_;
};

Tiago::Tiago() : local_nh_("~"), replan_(false)
{
    //=======================================================================
    // Subscribers
    //=======================================================================
}

void Tiago::run()
{
    ros::AsyncSpinner spinner(4);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveGroupInterface move_group_arm_torso("arm_torso");

    move_group_arm_torso.setPlannerId("RRTConnectkConfigDefault");

    auto pcm = planning_scene_monitor::PlanningSceneMonitorPtr(
        new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    planning_scene_monitor::LockedPlanningSceneRO planning_scene(pcm);

    // Open the gripper before starting
    moveit::planning_interface::MoveItErrorCode success =
        moveit::planning_interface::MoveItErrorCode::FAILURE;

    ros::Rate loop_rate(10);

    //=======================================================================
    // Replanning if needed
    //=======================================================================
    replan_ = true;
    std::string arm = "arm_torso";

    while (ros::ok())
    {
        if (replan_)
        {
            replan_ = true;

            // Pregrasp
            geometry_msgs::PoseStamped target_pose;

            target_pose.header.frame_id = "base_link";

            target_pose.pose.position.x = 0.59;
            target_pose.pose.position.y = -0.39;
            target_pose.pose.position.z = 1.15;
            target_pose.pose.orientation.x = -0.011;
            target_pose.pose.orientation.y = 0.99;
            target_pose.pose.orientation.z = -0.027;
            target_pose.pose.orientation.w = -0.02;
            move_group_arm_torso.setPoseReferenceFrame("base_link");
            move_group_arm_torso.setGoalTolerance(0.05);
            move_group_arm_torso.setGoalOrientationTolerance(0.05);
            move_group_arm_torso.setPoseTarget(target_pose);
            move_group_arm_torso.setApproximateJointValueTarget(target_pose, "gripper_link");

            success = move_group_arm_torso.plan(my_plan);

            if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {

                move_group_arm_torso.execute(my_plan);
            }
            if (!success)
            {
                continue;
            }
            else
            {
                break;
            }
            loop_rate.sleep();
            sleep(1.0);
        }
        // std::cout << "Waiting the user to move close."<< std::endl;
        loop_rate.sleep();
    }

    spinner.stop();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tiago_moveit_demo");

    Tiago demo;
    demo.run();

    return 0;
}