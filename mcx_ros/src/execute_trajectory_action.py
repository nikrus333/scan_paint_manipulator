#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import ExecuteTrajectoryArray
from math import pi, asin, cos, sin, acos, atan2, sqrt, radians
from typing import Union
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
from example_interfaces.srv import OpenClose
from example_interfaces.msg import TrajectoryMsg, CommandPose, PoseMsg

import motorcortex
import math
import time
from libs.motion_program import Waypoint, MotionProgram, PoseTransformer
from libs.robot_command import RobotCommand
from libs.system_defs import InterpreterStates
from paint_lidar.lidar_utils.gcode import Gcode, MOVC, MOVL
from paint_lidar.lidar_utils.trajectory import euler_from_quaternion, quaternion_from_euler

# from robot_control.motion_program import Waypoint, MotionProgram, PoseTransformer
# from robot_control.robot_command import RobotCommand
# from robot_control.system_defs import InterpreterStates
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os


class ExecuteTrajectoryAction(Node):

    def __init__(self):
        super().__init__('execute_trajectory_action')
        self._action_srv = ActionServer(
            self, ExecuteTrajectoryArray, 'execute_trajectory', self.execute_callback)
        # self.publisher_ = self.create_publisher(Bool, '/nozzle_close_open', 1)

        # /////////////////////////////////////////////
        self._client_nozzle = self.create_client(OpenClose, "service_nozzle")

        while not self._client_nozzle.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service_nozzle not available, waiting again...')

        self._req_open = OpenClose.Request()
        # /////////////////////////////////////////////

        self.__lastMsg = None

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
        self.robot = RobotCommand(self.req, self.motorcortex_types)

        if self.robot.engage():
            self.get_logger().info('Robot is at Engage')
        else:
            self.get_logger().info('Failed to set robot to Engage')
            return
        self.robot.reset()

    def check_point(self, req, motorcortex_types, point):
        pose_tf = PoseTransformer(req, motorcortex_types)
        #Точки в метрах, углы в радианах
        cart_coord=[point[0], point[1], point[2], point[3], point[4], point[5]]
        ref_joint_coord_rad=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ptf = pose_tf.calcCartToJointPose(cart_coord, ref_joint_coord_rad)
        angle_jointpose = []
        answer = False
        for i in range (0, 6):
            an = ptf.jointpose.coordinates[i]
            angle_jointpose.append(an)
        for i in range(len(angle_jointpose)):
            if angle_jointpose[i] == 0:
                break
            else:
                answer = True      
        return answer

    # /////////////////////////////////////////////
    def send_request(self, status: bool):
        self._req_open.status = status
        self.future = self._client_nozzle.call_async(self._req_open)
        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()
    # /////////////////////////////////////////////

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing trajectory...")

        result = ExecuteTrajectoryArray.Result()
        feedback_msg = ExecuteTrajectoryArray.Feedback()
        trajectory = goal_handle.request.trajectory_array #[TrajectoryMsg]
        vel = goal_handle.request.velocity
        acceleration = goal_handle.request.acceleration
        rotational_velocity = goal_handle.request.rotational_velocity
        rotational_acceleration = goal_handle.request.rotational_acceleration
        type_traject = goal_handle.request.type_traject
        print(trajectory)
        # print(f"poses_array: {poses_array}")
        type_traject_bool = False
        rotate = False
        buff_point = None 
        # Количество траектирий в массива
        if len(trajectory) > 2:
            rotate = False
        if type_traject == 'color':
            type_traject_bool = True
        # Проход по кол-ву траекторий
        for count_line, commands in enumerate(trajectory):
            type_traject_bool = False
            if type_traject == 'color':
                type_traject_bool = True
            motion_program = MotionProgram(self.req, self.motorcortex_types)
            points = []
            # print("trajectory:", trajectory)
            # Проход по командам внутри траектории
            for i, pose in enumerate(commands.command):
                if pose.type == "MOVL":
                    points = []
                    x = pose.start.x
                    y = pose.start.y
                    z = pose.start.z
                    nx = pose.start.nx
                    ny = pose.start.ny
                    nz = pose.start.nz
                    points.append(Waypoint([x, y, z, nz, ny, nx]))
                    
                    joint_params = self.joint_subscription.read()
                    value = joint_params[0].value
                    reference_joint_coord = value
                    motion_program.addMoveL(
                        points, vel, acceleration, rotational_velocity, rotational_acceleration, reference_joint_coord)
                    
                if pose.type == "MOVC":
                    points = []
                    x = pose.start.x
                    y = pose.start.y
                    z = pose.start.z
                    nx = pose.start.nx
                    ny = pose.start.ny
                    nz = pose.start.nz
                    points.append(Waypoint([x, y, z, nz, ny, nx]))
                    
                    x = pose.middle.x
                    y = pose.middle.y
                    z = pose.middle.z
                    nx = pose.middle.nx
                    ny = pose.middle.ny
                    nz = pose.middle.nz
                    points.append(Waypoint([x, y, z, nz, ny, nx]))

                    x = pose.end.x
                    y = pose.end.y
                    z = pose.end.z
                    nx = pose.end.nx
                    ny = pose.end.ny
                    nz = pose.end.nz
                    points.append(Waypoint([x, y, z, nz, ny, nx]))
                    
                    joint_params = self.joint_subscription.read()
                    value = joint_params[0].value
                    reference_joint_coord = value
                    motion_program.addMoveC(waypoint_list=points, angle=radians(pose.angle), velocity=0.025)

            # //////////////////////////////////////////////
            if rotate and count_line == 2 and type_traject == 'color':
                buff_point = points[0]
            # //////////////////////////////////////////////

            # motion_program.addMoveL(
            #     points, vel, acceleration, rotational_velocity, rotational_acceleration)
            # motion_program.addMoveL(points, vel, acceleration)
            type_traject_bool = commands.command[0].paint
            
            motion_program.send("test1").get()
            if self.robot.play(wait_time=0.1) is InterpreterStates.MOTION_NOT_ALLOWED_S.value:
                print('Robot is not at a start position, moving to the start')
                if self.robot.moveToStart(200):
                    print('Robot is at the start position')
                else:
                    raise Exception('Failed to move to the start position')
                self.robot.play(0.1)
                
            self.send_request(type_traject_bool)
            
            while self.robot.getState() is InterpreterStates.PROGRAM_IS_DONE.value:
                time.sleep(0.1)
                print('Waiting for the program to start, robot state: {}'.format(
                    self.robot.getState()))
                
            flag = True
            while self.robot.getState() is InterpreterStates.PROGRAM_RUN_S.value:
                # if flag:
                #     self.send_request(type_traject_bool)
                #     flag = False
                params = self.subscription.read()
                value = params[0].value
                # trans, euler = value[:3], value[3:]
                feedback_msg.robot_state = value
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.01)
                # print('Playing, robot state: {}'.format(self.robot.getState()))
            # msg = Bool()
            # msg.data = False
            # self.publisher_.publish(msg)
            type_traject_bool = False
            self.send_request(type_traject_bool)
            self.robot.reset()

            # /////////////////////////////////////////////
            if rotate and type_traject == 'color':
                if count_line == 1:
                    self.RotateGrip(True)
                if count_line == len(trajectory) - 1:
                    self.RotateGrip(False, [buff_point])
            
            if count_line == len(trajectory) - 1 and type_traject == 'color':
                self.MoveToStartPoint()
            # ////////////////////////////////////////////

        self.robot.reset()
        goal_handle.succeed()
        result.success = True
        self.get_logger().info("Trajectory done!")
        return result
    
    def RotateGrip(self, mode: bool, points = None):        
        print("Rotate")
        motion_program = MotionProgram(self.req, self.motorcortex_types)

        if points is not None:
            motion_program.addMoveL(points, velocity=0.008, acceleration=0.02)
            motion_program.send("rotate").get()  
            if self.robot.play() is InterpreterStates.MOTION_NOT_ALLOWED_S.value:
                print('Robot is not at a start position, moving to the start')
                if self.robot.moveToStart(10):
                    print('Robot is at the start position')
                else:
                    raise Exception('Failed to move to the start position')
            self.robot.reset()

        div = 10
        point_rotate = []
        joint_params = self.joint_subscription.read()
        value = joint_params[0].value
        point_rotate.append(Waypoint(value))

        for i in range(div):
            joint4 = 3.14 / div
            joint5 = -3.14 / div
            joint6 = -3.14 / div

            if not mode:
                joint4 *= -1
                joint5 *= -1
                joint6 *= -1

            point_rotate.append(Waypoint([value[0], value[1], value[2], value[3] + joint4 * (i + 1), value[4] + joint5 * (i + 1), value[5] + joint6 * (i + 1)]))
        motion_program.addMoveJ(waypoint_list=point_rotate, rotational_velocity=0.2, rotational_acceleration=1) 

        motion_program.send("rotate").get()  
        
        if self.robot.play() is InterpreterStates.MOTION_NOT_ALLOWED_S.value:
            print('Robot is not at a start position, moving to the start')
            if self.robot.moveToStart(10):
                print('Robot is at the start position')
            else:
                raise Exception('Failed to move to the start position')
        self.robot.play()

        while self.robot.getState() is InterpreterStates.PROGRAM_IS_DONE.value:
            time.sleep(0.1)
            print('Waiting for the program to start, robot state: {}'.format(
                self.robot.getState()))
            
        while self.robot.getState() is InterpreterStates.PROGRAM_RUN_S.value:
            params = self.subscription.read()
            value = params[0].value
            time.sleep(0.01)

        self.robot.reset()
    
    def MoveToStartPoint(self):
        motion_program = MotionProgram(self.req, self.motorcortex_types)    
        point = []
        x = 0.640
        y = -0.039
        z = 0.6
        point.append(Waypoint([x, y, z, math.radians(90), math.radians(0), math.radians(90)]))
        motion_program.addMoveL(point, velocity=0.05, acceleration=0.1)

        motion_program.send("move_to_start_point").get() 
        if self.robot.play() is InterpreterStates.MOTION_NOT_ALLOWED_S.value:
            print('Robot is not at a start position, moving to the start')
            if self.robot.moveToStart(10):
                print('Robot is at the start position')
            else:
                raise Exception('Failed to move to the start position')
        
        self.robot.reset()


def main(args=None):
    rclpy.init(args=args)
    move_manip_action_srv = ExecuteTrajectoryAction()
    rclpy.spin(move_manip_action_srv)


if __name__ == '__main__':
    main()
