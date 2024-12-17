// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Gazebo Attacher
#ifdef GAZEBO_LINK_ATTACHER
#include <gazebo_ros_link_attacher/Attach.h>
#endif

// Services
#ifdef GAZEBO_LINK_ATTACHER
gazebo_ros_link_attacher::Attach attach_srv_;
ros::ServiceClient attach_client_, detach_client_;
#endif

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

void attach1(moveit::planning_interface::MoveGroupInterface &move_group_atc)
{

    // Now, let's attach the collision object to the robot.
    move_group_atc.attachObject("target_box", "panda_leftfinger",
                                {"panda_rightfinger", "panda_leftfinger",
                                 "panda_hand"
                                 "link"});
#ifdef GAZEBO_LINK_ATTACHER
    attach_srv_.request.model_name_1 = "target_box";
    attach_srv_.request.link_name_1 = "link";

    if (attach_client_.call(attach_srv_))
        ROS_INFO("Successfully attached");
    else
        ROS_WARN("NOT attached");
#endif
}

void detach1(moveit::planning_interface::MoveGroupInterface &move_group_atc)
{
    // Now, let's detach the collision object from the robot.
    ROS_INFO_NAMED("tutorial", "Detach the object to the robot");
    move_group_atc.detachObject("target_box");

#ifdef GAZEBO_LINK_ATTACHER
    if (detach_client_.call(attach_srv_))
        ROS_INFO("Successfully detached");
    else
        ROS_WARN("NOT detached");
#endif
}

void openGripper(trajectory_msgs::JointTrajectory &posture)
{

    /* Add both finger joints of panda robot. */
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(2);
}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{

    /* Add both finger joints of panda robot. */

    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(2);
}

void pick(moveit::planning_interface::MoveGroupInterface &move_group)
{

    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> pre_grasps;
    pre_grasps.resize(1);

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    pre_grasps[0].grasp_pose.header.frame_id = "world";
    tf2::Quaternion orientation;
    orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
    pre_grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    pre_grasps[0].grasp_pose.pose.position.x = 0.202;
    pre_grasps[0].grasp_pose.pose.position.y = 0.032;
    pre_grasps[0].grasp_pose.pose.position.z = 1.3;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    pre_grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
    /* Direction is set as negative z axis */
    pre_grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    pre_grasps[0].pre_grasp_approach.min_distance = 0.04;
    pre_grasps[0].pre_grasp_approach.desired_distance = 0.05;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(pre_grasps[0].pre_grasp_posture);

    // Set support surface as table.
    move_group.setSupportSurfaceName("table");
    move_group.setGoalJointTolerance(0.1);
    // Call pick to pick up the object using the grasps given
    move_group.pick("target_box", pre_grasps);
    // END_SUB_TUTORIAL
}

void pick2(moveit::planning_interface::MoveGroupInterface &move_group2)
{

    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    grasps[0].grasp_pose.header.frame_id = "world";
    tf2::Quaternion orientation;
    orientation.setRPY(tau / 2, 0, tau / 8);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.05;
    grasps[0].grasp_pose.pose.position.y = 0.615;
    grasps[0].grasp_pose.pose.position.z = 0.88;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
    /* Direction is set as negative z axis */
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.04;
    grasps[0].pre_grasp_approach.desired_distance = 0.225;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.001;
    grasps[0].post_grasp_retreat.desired_distance = 0.002;

    closedGripper(grasps[0].grasp_posture);

    // Set support surface as table1.
    move_group2.setSupportSurfaceName("table");
    move_group2.setGoalJointTolerance(1);
    // Call pick to pick up the object using the grasps given
    move_group2.pick("target_box", grasps);
    // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface &group)
{

    // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
    // a single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    // +++++++++++++++++++++++++++
    place_location[0].place_pose.header.frame_id = "world";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, -tau / 8); // A quarter turn
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* For place location, we set the value to the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = 0.2;
    place_location[0].place_pose.pose.position.y = -0.5;
    place_location[0].place_pose.pose.position.z = 0.825;

    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "world";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.04;
    place_location[0].pre_place_approach.desired_distance = 0.10;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "world";
    /* Direction is set as positive z axis */
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.001;
    place_location[0].post_place_retreat.desired_distance = 0.002;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    group.setSupportSurfaceName("table");
    // Call place to place the object using the place locations given.
    group.place("target_box", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);

    // Add the table where the coke will originally be kept.
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "world";

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.8;
    collision_objects[0].primitives[0].dimensions[1] = 1.58;
    collision_objects[0].primitives[0].dimensions[2] = 0.72;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.365;
    collision_objects[0].primitive_poses[0].orientation.z = 0;

    collision_objects[0].operation = collision_objects[0].ADD;

    collision_objects[1].id = "wall";
    collision_objects[1].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.1;
    collision_objects[1].primitives[0].dimensions[1] = 2.5;
    collision_objects[1].primitives[0].dimensions[2] = 2.5;

    /* Define the pose of the object. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = -0.54;
    collision_objects[1].primitive_poses[0].position.y = 0;
    collision_objects[1].primitive_poses[0].position.z = 1.25;
    collision_objects[1].primitive_poses[0].orientation.z = 0;

    collision_objects[1].operation = collision_objects[1].ADD;

    collision_objects[2].id = "cabinet";
    collision_objects[2].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.5;
    collision_objects[2].primitives[0].dimensions[1] = 0.5;
    collision_objects[2].primitives[0].dimensions[2] = 2.5;

    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = -0.24;
    collision_objects[2].primitive_poses[0].position.y = 1.23;
    collision_objects[2].primitive_poses[0].position.z = 1.25;
    collision_objects[2].primitive_poses[0].orientation.z = 0;

    collision_objects[2].operation = collision_objects[2].ADD;

    // Define the object that we will be manipulating
    collision_objects[3].id = "target_box";
    collision_objects[3].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 0.05;
    collision_objects[3].primitives[0].dimensions[1] = 0.07;
    collision_objects[3].primitives[0].dimensions[2] = 0.07;

    /* Define the pose of the object. */
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0.052468;
    collision_objects[3].primitive_poses[0].position.y = 0.618629;
    collision_objects[3].primitive_poses[0].position.z = 0.76525;
    collision_objects[3].primitive_poses[0].orientation.w = 1.0;
    collision_objects[3].operation = collision_objects[3].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

//=======================================================================
// Services
//=======================================================================
#ifdef GAZEBO_LINK_ATTACHER
    attach_srv_.request.model_name_2 = "panda";
    attach_srv_.request.link_name_2 = "panda_leftfinger";
    // attach_srv_.request.model_name_2 = "panda_arm";
    // attach_srv_.request.link_name_2 = "panda_link7";

    attach_client_ = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_"
                                                                       "attacher_"
                                                                       "node/"
                                                                       "attach");

    detach_client_ = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_"
                                                                       "attacher_"
                                                                       "node/"
                                                                       "detach");
#endif

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(60.0);

    addCollisionObjects(planning_scene_interface);
    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    // to move to home position
    pick(group);
    ros::WallDuration(1.0).sleep();

    // to move to grasp position and grasp the object
    pick2(group);
    ros::WallDuration(1.0).sleep();
    attach1(group);
    ros::WallDuration(1.0).sleep();

    // to move to place position and place the object
    place(group);
    ros::WallDuration(1.0).sleep();
    detach1(group);
    ros::WallDuration(1.0).sleep();

    // to move to home position
    pick(group);

    ros::waitForShutdown();

    return 0;
}