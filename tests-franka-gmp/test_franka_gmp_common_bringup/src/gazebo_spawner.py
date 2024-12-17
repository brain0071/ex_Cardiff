#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Quaternion
import tf.transformations as tf
import random
import math
import os

# Constants for minimum and maximum values for x and y coordinates
X_MIN = -0.35
X_MAX = 0.4
Y_MIN = 0.9
Y_MAX = 1.0

# Minimum distance between objects
MIN_DISTANCE = 0.15  # Adjust as needed
NUM_OBJECTS = 2

# Define the maximum allowed runtime in seconds
max_runtime = 60  # This example stops the script after 60 seconds

# SDF strings for the cylinder and box
CYLINDER_SDF = """
<sdf version='1.6'>
  <model name='cylinder'>
    <pose>0 0 0 0 0 0</pose>
    <link name='link'>
      <pose>0 0 0 0 0 0</pose>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.23</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.23</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

BOX_SDF = """
<sdf version='1.6'>
  <model name='box'>
    <pose>0 0 0 0 0 0</pose>
    <link name='link'>
      <pose>0 0 0 0 0 0</pose>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.15 0.15 0.15</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.15 0.15 0.15</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient>
          <diffuse>0 0 1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

# List of models, including cylinder and box
models = [
    {"name": "cylinder", "sdf": CYLINDER_SDF},
    {"name": "box", "sdf": BOX_SDF},
    # Add other models here as needed
]


def generate_random_pose(z, roll, pitch, yaw):
    while True:
        pose = Pose()
        pose.position.x = random.uniform(X_MIN, X_MAX)
        pose.position.y = random.uniform(Y_MIN, Y_MAX)
        pose.position.z = z

        quaternion = euler_to_quaternion(roll, pitch, yaw)
        pose.orientation = quaternion

        if is_pose_valid(pose):
            return pose


def is_pose_valid(pose):
    # Check distance to all previously spawned objects
    for existing_pose in spawned_poses:
        distance = math.sqrt(
            (pose.position.x - existing_pose.position.x) ** 2
            + (pose.position.y - existing_pose.position.y) ** 2
        )
        if distance < MIN_DISTANCE:
            return False
    return True


def spawn_model(sdf, model_name, pose):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    try:
        spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        # Check if model_name already exists, add a numeric suffix if needed
        original_model_name = model_name
        suffix = 1
        while model_name in spawned_model_names:
            model_name = f"{original_model_name}_{suffix}"
            suffix += 1

        spawn_sdf(model_name, sdf, "", pose, "world")
        rospy.loginfo("Spawned model '{}' successfully.".format(model_name))
        spawned_model_names.append(model_name)  # Add the spawned model name to the list
        spawned_poses.append(pose)  # Add the spawned pose to the list
    except rospy.ServiceException as e:
        rospy.logerr("Spawn service call failed: {}".format(e))


def euler_to_quaternion(roll, pitch, yaw):
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(*quaternion)


if __name__ == "__main__":
    try:
        rospy.init_node("spawn_sdf_model_node", anonymous=True)

        start_time = rospy.Time.now()  # Start time for runtime limit

        spawned_model_names = []  # List to store the names of spawned models
        spawned_poses = []  # List to store the poses of spawned objects

        # Ensure the script stops after max_runtime seconds
        while not rospy.is_shutdown():
            # Check if the maximum runtime has been exceeded
            current_time = rospy.Time.now()
            elapsed_time = (current_time - start_time).to_sec()
            if elapsed_time >= max_runtime:
                rospy.loginfo(
                    f"Maximum runtime ({max_runtime} seconds) exceeded. Exiting..."
                )
                break

            num_objects = NUM_OBJECTS  # Number of objects to spawn

            z = 0.94  # Adjust the height as needed
            roll = 0.0  # in radians
            pitch = 0.0  # in radians
            yaw = 0.0

            # Spawn the specified number of objects
            for i in range(num_objects):
                # Generate random pose for the object ensuring it's not too close to existing objects
                random_pose = generate_random_pose(z, roll, pitch, yaw)
                model = models[i]
                model_name = model["name"]

                # Call the function to spawn the model
                spawn_model(model["sdf"], model_name, random_pose)

            # Sleep for a short duration before checking the runtime again
            rospy.sleep(1.0)

    except rospy.ROSInterruptException:
        rospy.loginfo("Script interrupted by ROS.")
        pass

    # Stop rospy.spin() and shutdown the node
    rospy.signal_shutdown("Script completed execution.")
