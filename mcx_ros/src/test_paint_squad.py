#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from example_interfaces.srv import OpenClose
from rclpy.callback_groups import ReentrantCallbackGroup

import motorcortex
import math
import time
from math import sin, cos
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
        self._leadSize  = 0.005
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
            self.req, self.sub = motorcortex.connect('wss://192.168.2.100:5568:5567', self.motorcortex_types, parameter_tree,
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

    def run(self):
        mode_work = self.chosseMission()
        match mode_work:
            case 1:
                self.missionSquad()
            case 2:
                self.missionT()
            case 3:
                self.missionChess()
        self.MoveToStartPoint()
    
    def chosseMission(self) -> int:
        while True:
            print("Enter mission")
            print("1 - Squad")
            print('2 - Letter \"T\"')
            print("3 - Chess")
            chosse_start_manip = input()
            if chosse_start_manip.isdigit():
                chosse_start_manip = int(chosse_start_manip)
            if (chosse_start_manip == 1) or (chosse_start_manip == 2) or (chosse_start_manip == 3):
                return chosse_start_manip
            else:
                print('Некорректный ввод')
    
    def missionChess(self):
        # Settings
        width = 0.08 - self._leadSize
        height = width
        leadSize = self._leadSize
        n = 2 # rows
        m = 3 # colums
        
        for i in range(n):
            for j in range(m):
                print("i:", i, "j:", j)
                if i % 2 == 1 and j % 2 == 0: continue
                if i % 2 == 0 and j % 2 == 1: continue
                pose = Pose()
                pose.position.x = 0.700 + self._offset.x - height * i
                pose.position.y = 0.0 + self._offset.y - width * j
                pose.position.z = self._dist_paint + self._tool_leng + self._offset.z
                q = quaternion_from_euler(math.radians(180), math.radians(0), math.radians(90))
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                array_list = self.createMatrixPaint(pose, width=width, height=height, leadSize=leadSize)
                print("array_list:", array_list)
                for k, line in enumerate(array_list):
                    print("line:", k + 1)
                    self.MovePoints(line, self._vel, self._acc, self.req, self.motorcortex_types)  
        pass
    
    def missionT(self):
        # Settings
        height = self._leadSize * 2
        width = 0.15
        connect = True
        # ------
        pose = Pose()
        pose.position.x = 0.7 + self._offset.x
        pose.position.y = 0.0 + self._offset.y
        pose.position.z = self._dist_paint + self._tool_leng + self._offset.z
        q = quaternion_from_euler(math.radians(180), math.radians(0), math.radians(90))
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        array_list = self.createMatrixPaint(pose, width=width, height=height, leadSize=self._leadSize, flag=False)
        print("len", len(array_list))
        for i, line in enumerate(array_list):
            print("line:", i + 1)
            self.MovePoints(line, self._vel, self._acc, self.req, self.motorcortex_types)  

        # |
        pose = Pose()
        pose.position.x = 0.7 + self._offset.x - (height if connect else 0)
        pose.position.y = (-width-height)/2 + self._offset.y
        pose.position.z = self._dist_paint + self._tool_leng + self._offset.z
        q = quaternion_from_euler(math.radians(180), math.radians(0), math.radians(180))
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        array_list = self.createMatrixPaint(pose, width=width - (height if connect else 0), height=height, leadSize=self._leadSize, flag=True)
        print("len", len(array_list))
        for i, line in enumerate(array_list):
            print("line:", i + 1)
            self.MovePoints(line, self._vel, self._acc, self.req, self.motorcortex_types)  
        pass
    
    def missionSquad(self):
        # Settings
        width = 0.100
        height = 0.100
        leadSize = self._leadSize
        
        pose = Pose()
        pose.position.x = 0.600 + self._offset.x
        pose.position.y = 0.0 + self._offset.y
        pose.position.z = self._dist_paint + self._tool_leng + self._offset.z
        q = quaternion_from_euler(math.radians(180), math.radians(0), math.radians(90))
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        array_list = self.createMatrixPaint(pose, width=width, height=height, leadSize=leadSize)
        print("array_list:", array_list)
        for i, line in enumerate(array_list):
            print("line:", i + 1)
            self.MovePoints(line, self._vel, self._acc, self.req, self.motorcortex_types)  
        pass

    def createMatrixPaint(self, pose: Pose, width: float, height: float, leadSize: float, flag=False) -> list:
        """
        pose - start pose\n
        width - width painting (m)\n
        height - height painting (m)\n
        leadSize - radious nozzle spraying (m)\n
        """
        array_list = []
        n = int(width / leadSize)
        m = int(height / leadSize)
        roll, pitch, yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        pi = math.pi
        for i in range(m):
            array_slice = []
            for j in range(n):
                new_pose = Pose()
                new_pose.orientation = pose.orientation
                y = (leadSize * j) + leadSize/2 if i % 2 == 0 else leadSize * (n+1-j-1) - leadSize/2
                x = leadSize * i + leadSize/2
                z = 0
                if not flag:
                    xyz = transform_point([x, y, z], [0, 0, 0], [roll-pi/2, pitch, yaw-pi])
                else:
                    xyz = transform_point([x, y, z], [0, 0, 0], [roll, pitch, yaw+pi/2])
                new_pose.position.x = pose.position.x - xyz[0]
                new_pose.position.y = pose.position.y - xyz[1]
                new_pose.position.z = pose.position.z - xyz[2]
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


def main(args=None):
    rclpy.init(args=args)
    node = TestPaintSquad()
    input("Wait enter for move to point 🐢")
    node.MoveToStartPoint()
    while KeyboardInterrupt:
        node.run()
    else:
        node.send_request(False)
        


if __name__ == '__main__':
    main()
