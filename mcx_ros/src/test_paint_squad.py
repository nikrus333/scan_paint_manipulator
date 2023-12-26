#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from example_interfaces.srv import OpenClose
from rclpy.callback_groups import ReentrantCallbackGroup

import motorcortex
import math
import time

from libs.motion_program import Waypoint, MotionProgram
from libs.robot_command import RobotCommand as robot_command
from libs.robot_command_old import RobotCommand as robot_command_old
from libs.system_defs import InterpreterStates

from paint_lidar.lidar_utils.trajectory import euler_from_quaternion, quaternion_from_euler
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os

version = "new"

class TestPaintSquad(Node):

    def __init__(self):
        super().__init__('execute_trajectory_action')

        self.__lastMsg = None
        self._vel = 0.05
        self._acc = 0.4
        self._tool_leng = 0.185
        self._dist_paint = 0.05
        self._leadSize  = 0.01
        self._offset = Point(x=.0, y=.0, z=-0.085)

        self.callback_group = ReentrantCallbackGroup()
        self._client_nozzle = self.create_client(OpenClose, "service_nozzle", callback_group=self.callback_group)
        self._client_nozzle_connect = True
        attempted = 0
        while not self._client_nozzle.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service nozzle execute not available')
            attempted += 1
            if attempted >= 5:
                print("Couldn`t to connect to service nozzle")
                self._client_nozzle_connect = False
                break

        parameter_tree = motorcortex.ParameterTree()
        self.motorcortex_types = motorcortex.MessageTypes()
        license_file = os.path.join(
            get_package_share_directory('mcx_ros'), 'license', 'mcx.cert.pem')

        try:
            self.req, self.sub = motorcortex.connect('wss://192.168.5.151:5568:5567', self.motorcortex_types, parameter_tree,
                                                     timeout_ms=1000, certificate=license_file,
                                                     login="admin", password="vectioneer")
            self.subscription = self.sub.subscribe(
                ['root/ManipulatorControl/fkActualToolCoords/toolCoordinates'], 'group1', 5)
            self.joint_subscription = self.sub.subscribe(
                ['root/ManipulatorControl/fkActualToolCoords/jointPositions'], 'group2', 5)
            self.subscription.get()

        except Exception as e:
            self.get_logger().info(license_file)
            self.get_logger().info(f"Failed to establish connection: {e}")
            return
        
        if version == "new":
            self.robot = robot_command(self.req, self.motorcortex_types)
        if version == "old":
            self.robot = robot_command_old(self.req, self.motorcortex_types)
    
        if self.robot.engage():
            self.get_logger().info('Robot is at Engage')
        else:
            self.get_logger().info('Failed to set robot to Engage')
            return
        
        self.robot.reset()
        self.run()

    def run(self):
        input("Wait enter for move to point 🐢")
        self.MoveToStartPoint()
        input("Wait enter for start program 🐢")
        self.paintSquad()
        self.MoveToStartPoint()
        pass
    
    def paintSquad(self):
        pose = Pose()
        pose.position.x = 0.600 + self._offset.x
        pose.position.y = 0.0 + self._offset.y
        pose.position.z = self._dist_paint + self._tool_leng + self._offset.z
        q = quaternion_from_euler(math.radians(180), math.radians(0), math.radians(90))
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        array_list = self.createMatrixPaint(pose, width=0.15, height=0.10, leadSize=self._leadSize)
        print("array_list:", array_list)
        for i, line in enumerate(array_list):
            print("line:", i + 1)
            self.MovePoints(line, self._vel, self._acc, self.req, self.motorcortex_types)  
        pass

    def createMatrixPaint(self, pose: Pose, width: float, height: float, leadSize: float) -> list:
        """
        pose - start pose\n
        width - width painting (m)\n
        height - height painting (m)\n
        leadSize - radious nozzle spraying (m)\n
        """
        array_list = []
        n = int(width / leadSize) + 1
        m = int(height / leadSize) + 1
        for i in range(m):
            array_slice = []
            for j in range(n):
                new_pose = Pose()
                new_pose.orientation = pose.orientation
                new_pose.position.x = pose.position.x - self._leadSize * i
                new_pose.position.y = pose.position.y - self._leadSize * j if i % 2 == 0 else pose.position.y - self._leadSize * (n-j-1)
                new_pose.position.z = pose.position.z
                array_slice.append(new_pose)
            array_list.append(array_slice)
        return array_list

    def send_request(self, status: bool) -> OpenClose.Response:
        print(f"Send {status}")
        request = OpenClose.Request()
        request.status = status
        self.future = self._client_nozzle.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def MoveToStartPoint(self):
        motion_program = MotionProgram(self.req, self.motorcortex_types)    
        point = []
        x = 0.600
        y = -0.0
        z = self._dist_paint + self._tool_leng + 0.1
        point.append(Waypoint([x, y, z, math.radians(90), math.radians(0), math.radians(180)]))
        motion_program.addMoveL(point, velocity=self._vel, acceleration=self._acc)

        motion_program.send("move_to_start_point").get() 
        if self.robot.play() is InterpreterStates.MOTION_NOT_ALLOWED_S.value:
            print('Robot is not at a start position, moving to the start')
            if self.robot.moveToStart(10):
                print('Robot is at the start position')
            else:
                raise Exception('Failed to move to the start position')
        self.robot.reset()

    def MovePoints(self, array_list: list[Pose], vel: float, acc: float, req: motorcortex.Request, motorcortex_types: motorcortex.MessageTypes, debug=False):
        motion_program = MotionProgram(req, motorcortex_types)    
        points = []
        if debug:
            print("count:", len(array_list))
            print("orient:", array_list[0].orientation)
        for point in array_list:
            roll, pitch, yaw = euler_from_quaternion([point.orientation.x, point.orientation.y, point.orientation.z, point.orientation.w])
            if debug:
                print(roll, yaw, pitch)
            points.append(Waypoint([point.position.x, point.position.y, point.position.z,
                                     roll, pitch, yaw]))
        motion_program.addMoveL(points, velocity=vel, acceleration=acc)
        motion_program.send("move_to_line").get() 
        if self.robot.play() is InterpreterStates.MOTION_NOT_ALLOWED_S.value:
            print('Robot is not at a start position, moving to the start')
            if self.robot.moveToStart(10):
                print('Robot is at the start position')
            else:
                raise Exception('Failed to move to the start position')
        
        # Nozzle on
        if self._client_nozzle_connect:
            self.send_request(True)
        else:
            print(True)

        self.robot.play()
        while self.robot.getState() is InterpreterStates.PROGRAM_IS_DONE.value:
            time.sleep(0.1)
            print('Waiting for the program to start, robot state: {}'.format(self.robot.getState()))

        while self.robot.getState() is InterpreterStates.PROGRAM_RUN_S.value:
            if version == "new":
                params = self.subscription.read()
                value = params[0].value
            time.sleep(0.01)
        
        # Nozzle off
        if self._client_nozzle_connect:
            self.send_request(False)
        else:
            print(False)

        self.robot.reset()


def main(args=None):
    rclpy.init(args=args)
    node = TestPaintSquad()
    # rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
