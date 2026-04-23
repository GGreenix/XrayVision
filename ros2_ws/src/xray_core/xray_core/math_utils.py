import math
from typing import Iterable, Sequence


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize(vector: Sequence[float]) -> list[float]:
    length = math.sqrt(sum(component * component for component in vector))
    if length == 0.0:
        return [0.0, 0.0, 0.0]
    return [component / length for component in vector]


def add_vectors(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [left + right for left, right in zip(a, b)]


def subtract_vectors(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [left - right for left, right in zip(a, b)]


def scale_vector(vector: Sequence[float], scale: float) -> list[float]:
    return [component * scale for component in vector]


def vector_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(a, b)))


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
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
    return (x, y, z, w)


def quaternion_to_matrix(quaternion: Sequence[float]) -> list[list[float]]:
    x, y, z, w = quaternion
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def rotation_matrix_from_euler(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    return quaternion_to_matrix(quaternion_from_euler(roll, pitch, yaw))


def transpose(matrix: Iterable[Iterable[float]]) -> list[list[float]]:
    rows = [list(row) for row in matrix]
    return [list(column) for column in zip(*rows)]


def matrix_vector_multiply(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> list[float]:
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def matrix_multiply(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    result = []
    for row in left:
        result_row = []
        for column in zip(*right):
            result_row.append(sum(left_value * right_value for left_value, right_value in zip(row, column)))
        result.append(result_row)
    return result

