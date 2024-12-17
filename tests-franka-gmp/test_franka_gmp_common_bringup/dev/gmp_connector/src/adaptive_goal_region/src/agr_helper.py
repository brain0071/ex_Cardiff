import csv
import heapq
import math
from typing import Tuple, Optional
from gazebo_msgs.msg import ModelStates
import rospy
import numpy as np

from scipy.spatial.transform import Rotation
from itertools import combinations

COLORS = [
    (0.1, 0.2, 0.3),
    (0.4, 0.5, 0.6),
    (0.7, 0.8, 0.9),
    (0.3, 0.1, 0.4),
    (0.5, 0.2, 0.6),
    (0.8, 0.3, 0.9),
    (0.2, 0.4, 0.1),
    (0.5, 0.6, 0.2),
    (0.8, 0.9, 0.3),
    (0.1, 0.3, 0.5),
    (0.4, 0.2, 0.7),
    (0.6, 0.1, 0.8),
    (0.9, 0.4, 0.2),
    (0.2, 0.7, 0.1),
    (0.5, 0.9, 0.3),
]
LINE_COLORS = [
    "red",
    "green",
    "blue",
    "orange",
    "purple",
    "brown",
    "pink",
    "black",
    "royalblue",
    "mediumblue",
    "darkblue",
    "navy",
    "steelblue",
]


def quaternions_to_euler(quaternions: np.ndarray) -> np.ndarray:
    quaternions_array = np.array(quaternions)
    rotation = Rotation.from_quat(quaternions_array)
    euler_angles = rotation.as_euler("xyz", degrees=False)
    euler_list = euler_angles.tolist()
    return np.array(euler_list)


def euler_to_quaternion(
    yaw: float, pitch: float, roll: float
) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


def quaternion_angular_distance(
    orientation1: np.ndarray, orientation2: np.ndarray
) -> float:
    orientation1 = orientation1 / np.linalg.norm(orientation1)
    orientation2 = orientation2 / np.linalg.norm(orientation2)
    dot = np.dot(orientation1, orientation2)
    dot = np.clip(dot, -1.0, 1.0)
    angle = 2 * np.arccos(np.abs(dot))
    return angle


def position_distance(position1: np.ndarray, positions: np.ndarray) -> float:
    return math.sqrt(
        (position1[0] - positions[0]) ** 2
        + (position1[1] - positions[1]) ** 2
        + (position1[2] - positions[2]) ** 2
    )


def max_distance_in_positions(positions: np.ndarray) -> float:
    max_dist = 0
    for point1, point2 in combinations(positions, 2):
        dist = position_distance(point1, point2)
        if dist > max_dist:
            max_dist = dist
    return max_dist


def closest_position_index(
    positions: np.ndarray, reference_position: np.ndarray
) -> int:
    points_array = np.array(positions)
    ref_array = np.array(reference_position)
    distances = np.sum((points_array - ref_array) ** 2, axis=1)
    min_index = np.argmin(distances)
    return int(min_index)


def n_closest_points_indices(points, target_point, n=3):
    distances = [
        (position_distance(point, target_point), index)
        for index, point in enumerate(points)
    ]
    closest_indices = heapq.nsmallest(n, distances)
    return [index for (_, index) in closest_indices]

def calculate_distance_2d(position1, position2):
    """Calculate Euclidean distance between two 2D positions (x, y)."""
    return math.sqrt(
        (position1.x - position2.x) ** 2 +
        (position1.y - position2.y) ** 2
    )

def get_model_states():
    """Get the positions of models from /gazebo/model_states topic."""
    model_states = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    return model_states

def get_distance_between_objects_2d(model_states, object1="cylinder", object2="box"):
    """Get the 2D distance between two objects given their names."""
    index1 = model_states.name.index(object1)
    index2 = model_states.name.index(object2)
    
    position1 = model_states.pose[index1].position
    position2 = model_states.pose[index2].position
    
    return calculate_distance_2d(position1, position2)



def save_agr_data_as_txt(
    data: np.ndarray, path: Optional[str] = None, unique: Optional[bool] = True
) -> None:
    temp = set()
    model_states = get_model_states()
    distance = get_distance_between_objects_2d(model_states)
    with open(path, "w") as file:
        for pose in data:
            if unique and str(pose) in temp:
                continue
            elif unique:
                temp.add(str(pose))
            text = ""
            for i in pose:
                text += "{:.5f} ".format(i)
            text += "{:.5f} ".format(distance)
            file.write(text + "\n")


def rotate_orientation(orientation: np.ndarray, angle_degrees: int = 90) -> np.ndarray:
    angle_radians = np.pi * (angle_degrees / 180.0)
    rotation = Rotation.from_euler("z", angle_radians)
    original_orientation = Rotation.from_quat(orientation)
    transformed_quaternion = rotation * original_orientation
    return transformed_quaternion.as_quat()
