import rclpy
import numpy as np
from math import * 

def transform_point(point, translation, rotation_angles):
    translation_vector = np.array(translation)
    rotation_matrix = euler_rotation_matrix(rotation_angles)
    point_vector = np.array(point)
    translated_point = point_vector + translation_vector
    rotated_point = np.around(np.dot(rotation_matrix, translated_point), 5)
    return rotated_point

def rotate_origin(point, origin, rotation_angles):
    rotation_matrix = euler_rotation_matrix(rotation_angles)
    point_vector = np.array(point)
    translated_point = point_vector - origin
    rotated_point = np.around(np.dot(rotation_matrix, translated_point), 5)
    rotated_point += origin
    return rotated_point

def euler_rotation_matrix(rotation_angles):
    # Создание матрицы поворота по углу X
    rotation_matrix_x = rotx(float(rotation_angles[0]))
    # Создание матрицы поворота по углу Y
    rotation_matrix_y = roty(float(rotation_angles[1]))
    # Создание матрицы поворота по углу Z
    rotation_matrix_z = rotz(float(rotation_angles[2]))
    rotation_matrix = np.dot(rotation_matrix_z, np.dot(rotation_matrix_y, rotation_matrix_x))
    return rotation_matrix

def rotx(alpha):
    # Создание матрицы поворота по углу X
    rotation_matrix_x = np.array([[1, 0, 0],
                                [0, np.cos(alpha), -np.sin(alpha)],
                                [0, np.sin(alpha), np.cos(alpha)]])
    return rotation_matrix_x

def roty(beta):
    # Создание матрицы поворота по углу Y
    rotation_matrix_y = np.array([[np.cos(beta), 0, np.sin(beta)],
                                [0, 1, 0],
                                [-np.sin(beta), 0, np.cos(beta)]])
    return rotation_matrix_y

def rotz(gamma):
    # Создание матрицы поворота по углу Z
    rotation_matrix_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                                [np.sin(gamma), np.cos(gamma), 0],
                                [0, 0, 1]])
    return rotation_matrix_z
     

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw