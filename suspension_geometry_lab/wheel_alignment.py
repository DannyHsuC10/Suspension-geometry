"""Wheel-only camber installation, applied after zero-camber construction."""
import math

import numpy as np


def installation_matrix(camber_deg, corner):
    """Vehicle-frame wheel rotation: negative camber means top inward on both sides."""
    angle = math.radians(camber_deg)*(1 if corner.endswith("R") else -1)
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def rotate_wheel_points(points, center, matrix):
    if np.array_equal(matrix, np.eye(3)):
        return np.array(points, copy=True)
    return center+(np.asarray(points)-center)@matrix.T


def loaded_contact(center, normal, radius):
    """Lowest point of the loaded-radius wheel center circle, not its tire shoulder."""
    down = np.array([0., 0., 1.])-normal[2]*normal
    length = np.linalg.norm(down)
    if length < 1e-10:
        raise ValueError("輪胎中心圓近乎水平，無法定義唯一接地點。")
    return center-radius*down/length


def install_wheel(wheel, camber_deg, corner, radius):
    """Post-process display/pose geometry without feeding it into arm construction."""
    matrix = installation_matrix(camber_deg, corner)
    center = wheel["center"]
    wheel.update(static_camber_deg=camber_deg, installation_matrix=matrix,
                 construction_contact=center-np.array([0., 0., radius]),
                 contact=loaded_contact(center, matrix@np.array([0., 1., 0.]), radius))
    if camber_deg != 0.:
        for name in ("tire", "rim"):
            if name in wheel:
                shape = wheel[name]
                wheel["zero_camber_"+name] = dict(shape)
                for key in ("vertices", "outline"):
                    shape[key] = rotate_wheel_points(shape[key], center, matrix)
    return matrix
