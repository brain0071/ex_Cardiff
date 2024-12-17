#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the table where the coke will originally be kept.
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "world";

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1.4;
    collision_objects[0].primitives[0].dimensions[1] = 1.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.9;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.142;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.45;
    collision_objects[0].primitive_poses[0].orientation.z = 0;

    collision_objects[0].operation = collision_objects[0].ADD;

    collision_objects[1].id = "wall";
    collision_objects[1].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.01;
    collision_objects[1].primitives[0].dimensions[1] = 2.5;
    collision_objects[1].primitives[0].dimensions[2] = 2;

    /* Define the pose of the object. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = -0.615;
    collision_objects[1].primitive_poses[0].position.y = 0;
    collision_objects[1].primitive_poses[0].position.z = 1;
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
    collision_objects[2].primitives[0].dimensions[2] = 2;

    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = -0.24;
    collision_objects[2].primitive_poses[0].position.y = 1.23;
    collision_objects[2].primitive_poses[0].position.z = 1;
    collision_objects[2].primitive_poses[0].orientation.z = 0;

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(60.0);

    addCollisionObjects(planning_scene_interface);
    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();
}
