#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
import os
import yaml
import argparse

# Global variable to store model states
model_states = None

# Callback function to listen to /gazebo/model_states topic
def model_states_callback(msg):
    global model_states
    model_states = msg

# Initialize ROS node
rospy.init_node("gazebo_listener_node", anonymous=True)

# Subscribe to /gazebo/model_states topic
rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

# Wait for initial message from the topic
rospy.wait_for_message("/gazebo/model_states", ModelStates)

# Static YAML structure
yaml_data = {
    "fixed_frame_transforms": [
        {
            "child_frame_id": "world",
            "transform": {"rotation": [0, 0, 0, 1], "translation": [0, 0, 0]},
        }
    ],
    "name": "(noname)",
    "robot_model_name": "panda",
    "robot_state": {
        "joint_state": {
            "name": [
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
                "panda_finger_joint1",
                "panda_finger_joint2",
            ],
            "position": [
                1.62266,
                -1.4696,
                -1.875,
                -2.855,
                -2.852911,
                3.42766,
                1.768812,
                0.04,
                0.04,
            ],
        },
        "multi_dof_joint_state": {
            "joint_names": ["virtual_joint"],
            "transforms": [{"rotation": [0, 0, 0, 1], "translation": [0, 0.2, 0.725]}],
        },
    },
    "world": {
        "collision_objects": [
            {
                "id": "Table",
                "primitive_poses": [
                    {
                        "orientation": [0, 0, 0, 1],
                        "position": [0.0, 0.0, 0.71],
                    }
                ],
                "primitives": [
                    {
                        "dimensions": [0.8, 1.58, 0.019708],
                        "type": "box",
                    }
                ],
            },
            {
                "id": "Back",
                "primitive_poses": [
                    {
                        "orientation": [0, 0, 0, 1],
                        "position": [0, 1.165, 0.6],
                    }
                ],
                "primitives": [
                    {
                        "dimensions": [0.9, 0.01, 1.2],
                        "type": "box",
                    }
                ],
            },
            {
                "id": "Left",
                "primitive_poses": [
                    {
                        "orientation": [0, 0, 0, 1],
                        "position": [0.45, 0.965, 0.6],
                    }
                ],
                "primitives": [
                    {
                        "dimensions": [0.02, 0.4, 1.2],
                        "type": "box",
                    }
                ],
            },
            {
                "id": "Right",
                "primitive_poses": [
                    {
                        "orientation": [0, 0, 0, 1],
                        "position": [-0.45, 0.965, 0.6],
                    }
                ],
                "primitives": [
                    {
                        "dimensions": [0.02, 0.4, 1.2],
                        "type": "box",
                    }
                ],
            },
            {
                "id": "Bottom",
                "primitive_poses": [
                    {
                        "orientation": [0, 0, 0, 1],
                        "position": [0, 0.965, 0.03],
                    }
                ],
                "primitives": [
                    {
                        "dimensions": [0.88, 0.4, 0.06],
                        "type": "box",
                    }
                ],
            },
            {
                "id": "Top",
                "primitive_poses": [
                    {
                        "orientation": [0, 0, 0, 1],
                        "position": [0, 0.965, 1.19],
                    }
                ],
                "primitives": [
                    {
                        "dimensions": [0.88, 0.4, 0.02],
                        "type": "box",
                    }
                ],
            },
            {
                "id": "LowShelf",
                "primitive_poses": [
                    {
                        "orientation": [0, 0, 0, 1],
                        "position": [0, 0.965, 0.43],
                    }
                ],
                "primitives": [
                    {
                        "dimensions": [0.88, 0.4, 0.02],
                        "type": "box",
                    }
                ],
            },
            {
                "id": "HighShelf",
                "primitive_poses": [
                    {
                        "orientation": [0, 0, 0, 1],
                        "position": [0, 0.965, 0.8],
                    }
                ],
                "primitives": [
                    {
                        "dimensions": [0.88, 0.4, 0.02],
                        "type": "box",
                    }
                ],
            },
        ]
    },
}

# Define dynamic objects
dynamic_objects = {
    "Box": {
        "model_name": "box",
        "default_pose": {
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.00, 0.0, 1],
        },
    },
    "Cylinder": {
        "model_name": "cylinder",
        "default_pose": {
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1],
        },
    },
}

# Retrieve and update dynamic object poses if available
if model_states:
    for obj_name, obj_info in dynamic_objects.items():
        model_name = obj_info["model_name"]
        if model_name in model_states.name:
            index = model_states.name.index(model_name)
            pose = model_states.pose[index]
            dynamic_objects[obj_name]["pose"] = {
                "position": [pose.position.x, pose.position.y, pose.position.z],
                "orientation": [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
            }
        else:
            dynamic_objects[obj_name]["pose"] = obj_info["default_pose"]
else:
    for obj_name in dynamic_objects.keys():
        dynamic_objects[obj_name]["pose"] = dynamic_objects[obj_name]["default_pose"]

# Update YAML data with dynamic object information
for obj_name, obj_info in dynamic_objects.items():
    yaml_data["world"]["collision_objects"].append(
        {
            "id": obj_name,
            "primitives": [
                {
                    "type": "box" if obj_name == "Box" else "cylinder",
                    "dimensions": (
                        [0.15, 0.15, 0.15] if obj_name == "Box" else [0.23, 0.02]
                    ),
                }
            ],
            "primitive_poses": [
                {
                    "position": obj_info["pose"]["position"],
                    "orientation": obj_info["pose"]["orientation"],
                }
            ],
        }
    )

# Function to save YAML to a file
def save_yaml(yaml_data, save_path):
    with open(save_path, "w") as file:
        yaml.dump(yaml_data, file, default_flow_style=False)
    print(f"YAML file saved successfully to: {save_path}")

    # Shutdown ROS node
    rospy.signal_shutdown("YAML file saved")

# Parse command line arguments
parser = argparse.ArgumentParser(
    description="Create YML file for Robowflex"
)

parser.add_argument(
    "-o", "--output_dir", type=str, required=True, default="/home/furkanduman/dev/storage", help="The output dir"
)

args = parser.parse_args()

# Construct the path for saving scene data
save_path = os.path.join(args.output_dir, "panda.yml")
save_yaml(yaml_data, save_path)

# Spin ROS node to handle callbacks (will not block because we shut down after saving YAML)
rospy.spin()
