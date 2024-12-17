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

//!  OpenManipulatorMoveItHome class.
/*!
 * OpenManipulator MoveIt! Home.
 *
 */
class OpenManipulator
{
public:
    //! Constructor
    OpenManipulator();
    //! Function for waiting for user confirmation
    void waitForUserConfirmation(const std::string &next_task);
    //! Run
    void run();

private:
    // ROS
    ros::NodeHandle node_handler_, local_nh_;
    bool replan_;
};

OpenManipulator::OpenManipulator() : local_nh_("~"), replan_(false)
{ //=======================================================================
  // Subscribers //=======================================================================
}

void OpenManipulator::run()
{
    ros::AsyncSpinner spinner(4);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveGroupInterface move_group_right("arm");

    move_group_right.setPlannerId("RRTConnectkConfigDefault");

    auto pcm = planning_scene_monitor::PlanningSceneMonitorPtr(
        new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    planning_scene_monitor::LockedPlanningSceneRO planning_scene(pcm);

    // Open the gripper before starting
    moveit::planning_interface::MoveItErrorCode success =
        moveit::planning_interface::MoveItErrorCode::FAILURE;

    ros::Rate loop_rate(10); //=======================================================================
    // Replanning if needed //=======================================================================
    replan_ = true;

    while (ros::ok())
    {
        if (replan_)
        {
            replan_ = true;

            // Pregrasp
            geometry_msgs::PoseStamped target_pose;

            target_pose.header.frame_id = "world";
            target_pose.pose.position.x = 0.012;
            target_pose.pose.position.y = 0.0;
            target_pose.pose.position.z = 0.14;
            target_pose.pose.orientation.x = 0.0;
            target_pose.pose.orientation.y = 0.0;
            target_pose.pose.orientation.z = 0.0;
            target_pose.pose.orientation.w = 1.0;
            move_group_right.setPoseReferenceFrame("world");
            move_group_right.setGoalTolerance(0.05);
            move_group_right.setGoalOrientationTolerance(0.05);
            move_group_right.setPoseTarget(target_pose);
            move_group_right.setApproximateJointValueTarget(target_pose, "link5");

            success = move_group_right.plan(my_plan);

            if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {

                move_group_right.execute(my_plan);
            }
            loop_rate.sleep();
            sleep(1.0);

            if (!success)
                continue;
            break;
        }
        // std::cout << "Waiting the user to move close."<< std::endl;
        loop_rate.sleep();
    }

    spinner.stop();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OpenManipulator_moveit_home");

    OpenManipulator home;
    home.run();

    return 0;
}
