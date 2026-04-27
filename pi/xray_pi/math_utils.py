"""Vendored from ros2_ws/src/xray_core/xray_core/math_utils.py.

Kept identical so the localization math behaves the same on Pi and PC.
"""

import math
from typing import Iterable, Sequence


def normalize(vector: Sequence[float]) -> list[float]:
    length = math.sqrt(sum(c * c for c in vector))
    if length == 0.0:
        return [0.0, 0.0, 0.0]
    return [c / length for c in vector]


def add_vectors(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [l + r for l, r in zip(a, b)]


def matrix_vector_multiply(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> list[float]:
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def matrix_multiply(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    return [
        [sum(lv * rv for lv, rv in zip(row, col)) for col in zip(*right)]
        for row in left
    ]


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_to_matrix(q: Sequence[float]) -> list[list[float]]:
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def rotation_matrix_from_euler(roll: float, pitch: float, yaw: float):
    return quaternion_to_matrix(quaternion_from_euler(roll, pitch, yaw))
