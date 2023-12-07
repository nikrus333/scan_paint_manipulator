#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import ExecuteTrajectoryArray
from math import pi, asin, cos, sin, acos, atan2, sqrt, radians
from typing import Union
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

import motorcortex
import math
import time
from robot_control.motion_program import Waypoint, MotionProgram, PoseTransformer
from robot_control.robot_command import RobotCommand
from robot_control.system_defs import InterpreterStates
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os



def main(args=None):
    rclpy.init(args=args)
    node = Node('read_manip_node')

    parameter_tree = motorcortex.ParameterTree()
    motorcortex_types = motorcortex.MessageTypes()
    license_file = os.path.join(
        get_package_share_directory('mcx_ros'), 'license', 'mcx.cert.pem')

    try:
        req, sub = motorcortex.connect('wss://192.168.5.86:5568:5567', motorcortex_types, parameter_tree,
                                                    timeout_ms=1000, certificate=license_file,
                                                    login="admin", password="vectioneer")
        subscription = sub.subscribe(['root/Control/fkActualToolCoord/jointPositions'], 'group1', 5)
        subscription.get()
    except Exception as e:
        node.get_logger().info(license_file)
        node.get_logger().info(f"Failed to establish connection: {e}")
        return
    robot = RobotCommand(req, motorcortex_types)

    while True:
        params = subscription.read()
        print(params[0].value)
        #rclpy.spin_once(node)
        time.sleep(0.1)
    

if __name__ == '__main__':
    main()   