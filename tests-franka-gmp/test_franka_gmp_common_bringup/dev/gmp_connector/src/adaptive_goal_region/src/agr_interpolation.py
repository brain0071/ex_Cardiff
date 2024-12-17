from typing import Tuple

import numpy as np


from sklearn.decomposition import PCA
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures


from adaptive_goal_region.src.agr_helper import max_distance_in_positions
# mpl.use('macosx')


def converge_points_poly(points: np.ndarray, degree: int = 2):
    points = np.array(points)
    x = points[:, 0].reshape(-1, 1)
    y = points[:, 1]
    z = points[:, 2]
    poly = PolynomialFeatures(degree=degree)
    x_poly = poly.fit_transform(x)
    reg_y = LinearRegression().fit(x_poly, y)
    reg_z = LinearRegression().fit(x_poly, z)

    return reg_y, reg_z, poly


def generate_interpolation_edges(positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    points_array = np.array(positions)
    pca = PCA(n_components=1)
    pca.fit(points_array)
    line_direction = pca.components_[0]
    line_point = pca.mean_
    d = max_distance_in_positions(positions)
    t_min = -d / 2
    t_max = d / 2
    line_start = line_point + line_direction * t_min
    line_end = line_point + line_direction * t_max

    return line_start, line_end



