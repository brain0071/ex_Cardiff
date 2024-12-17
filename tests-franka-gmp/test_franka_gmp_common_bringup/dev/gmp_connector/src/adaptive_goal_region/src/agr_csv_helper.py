from typing import List

import numpy as np


def numpy_array_to_str(array: np.ndarray) -> str:
    array = [str(i) for i in array]
    return f"""['{"','".join(array)}']"""


def matrix_to_str(matrix: np.ndarray) -> str:
    matrix = [numpy_array_to_str(i) for i in matrix]
    return f"""[{",".join(matrix)}]"""


def generate_lines(poses: np.ndarray) -> List:
    ind = poses.shape[1] // 2
    contact_points = numpy_array_to_str(np.zeros(3))
    lines = ["pred_grasps_cam,scores,contact_pts\n"]
    for pose in poses:
        pose_1 = numpy_array_to_str(pose[:ind])
        pose_2 = numpy_array_to_str(pose[ind:])
        lines.append(f"\"{pose_1}\", 0.0, \"{contact_points}\"\n")
        lines.append(f"\"{pose_2}\", 0.0, \"{contact_points}\"\n")
    return lines


def save_agr_data_as_csv(adaptive_goal_region_data: np.ndarray):
    with open('agr_output', 'w', newline='') as file:
        for line in generate_lines(adaptive_goal_region_data):
            file.write(line)
