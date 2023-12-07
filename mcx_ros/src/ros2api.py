import motorcortex
import math
import time
from motion_program import Waypoint, MotionProgram
from robot_command import RobotCommand
from system_defs import InterpreterStates
import numpy as np
from ament_index_python.packages import get_package_share_directory
import threading
import rclpy
from rclpy.node import Node
from mcx_ros.srv import MoveSrv
from std_srvs.srv import SetBool
import os


def sendProgram(robot, motion_program):
    # send the program
    program_sent = motion_program.send("examp1").get()
    print(program_sent.status)
    robot_play_state = robot.play()
    # try to play the program
    if robot_play_state == InterpreterStates.PROGRAM_RUN_S.value:
        print("Playing program")
    elif robot_play_state == InterpreterStates.MOTION_NOT_ALLOWED_S.value:
        print("Can not play program, Robot is not at start")
        print("Moving to start")
        if robot.moveToStart(100):
            print("Move to start completed")
            robot_play_state_start = robot.play()
            if robot_play_state_start == InterpreterStates.PROGRAM_RUN_S.value:
                print("Playing program")
            elif robot_play_state_start == InterpreterStates.PROGRAM_IS_DONE.value:
                # pass
                print("Program is done")
            else:
                raise RuntimeError(
                    "Failed to play program, state: %s" % robot.getState())
        else:
            raise RuntimeError('Failed to move to start')
    elif robot_play_state == InterpreterStates.PROGRAM_IS_DONE.value:
        print("Program is done")
    else:
        raise RuntimeError("Failed to play program, state: %s" %
                           robot.getState())

    # waiting until the program is finished

    while robot.getState() is InterpreterStates.PROGRAM_RUN_S.value:
        time.sleep(0.1)


class ROSAPI(Node):
    def __init__(self) -> None:
        super().__init__("Ros2api")

        self.__lastMsg = None
        self.srv = self.create_service(
            MoveSrv, 'MoveL', self.moveL_callback)
        self.srv2 = self.create_service(
            MoveSrv, 'MoveJ', self.moveJ_callback)
        self.srv2 = self.create_service(
            SetBool, 'Gripper', self.gripper_callback)
        parameter_tree = motorcortex.ParameterTree()
        self.motorcortex_types = motorcortex.MessageTypes()
        license_file = os.path.join(
            get_package_share_directory('mcx_ros'), 'license', 'mcx.cert.pem')
        # Open request connection
        try:
            # req, sub = motorcortex.connect('wss://192.168.2.100:5568:5567', motorcortex_types, parameter_tree,
            #                                 timeout_ms=1000, certificate="applied_robotics.pem",
            #                                 login="root", password="vectioneer")
            self.req, self.sub = motorcortex.connect('wss://localhost:5568:5567', self.motorcortex_types, parameter_tree,
                                                     timeout_ms=1000, certificate=license_file,
                                                     login="admin", password="vectioneer")

            self.get_logger().info("Request connection is etablished")
        except Exception as e:
            self.get_logger().info(license_file)
            self.get_logger().info(f"Failed to establish connection: {e}")
            return
        self.robot = RobotCommand(self.req, self.motorcortex_types)

        if self.robot.engage():
            self.get_logger().info('Robot is at Engage')
        else:
            self.get_logger().info('Failed to set robot to Engage')
        motion_program_start = MotionProgram(self.req, self.motorcortex_types)
        start_position_jnt = Waypoint([math.radians(90.0), math.radians(0.0), math.radians(
            90.0), math.radians(0.0), math.radians(90.0), math.radians(0.0)])
        motion_program_start.addMoveJ([start_position_jnt], 0.3, 0.3)
        sendProgram(self.robot, motion_program_start)

        self.thread = threading.Thread(
            target=self.__spin, args=(self, ))
        self.thread.start()
        self.thread.join()

    def __spin(self, node):
        rclpy.spin(self)
        self.req.close()
        self.sub.close()
        node.destroy_node()
        rclpy.shutdown()

    def moveL_callback(self, req, res):
        points = []
        cmd1 = req.cmd1.tolist()
        cmd2 = req.cmd2.tolist()
        cmd3 = req.cmd3.tolist()
        cmd4 = req.cmd4.tolist()
        cmd5 = req.cmd5.tolist()
        cmd6 = req.cmd6.tolist()
        for i, _ in enumerate(cmd1):
            points.append(Waypoint(
                [cmd1[i], cmd2[i], cmd3[i], cmd4[i], cmd5[i], cmd6[i]]))
        motion_program = MotionProgram(self.req, self.motorcortex_types)
        motion_program.addMoveL(points, req.velocity, req.acceleration)
        self.get_logger().info("Incoming request to motionL")
        sendProgram(self.robot, motion_program)
        res.success = True
        return res

    def gripper_callback(self, req, res):
        if req.data == True:
            set_param_reply_msg = self.req.setParameter(
                'root/Control/dummyDouble', 1).get()
        else:
            set_param_reply_msg = self.req.setParameter(
                'root/Control/dummyDouble', 0).get()
        res.success = True
        return res

    def moveJ_callback(self, req, res):
        points = []
        cmd1 = req.cmd1.tolist()
        cmd2 = req.cmd2.tolist()
        cmd3 = req.cmd3.tolist()
        cmd4 = req.cmd4.tolist()
        cmd5 = req.cmd5.tolist()
        cmd6 = req.cmd6.tolist()
        for i, _ in enumerate(cmd1):
            points.append(Waypoint(
                [cmd1[i], cmd2[i], cmd3[i], cmd4[i], cmd5[i], cmd6[i]]))
        motion_program = MotionProgram(self.req, self.motorcortex_types)
        motion_program.addMoveJ(points, req.velocity, req.acceleration)
        self.get_logger().info("Incoming request to motionJ")
        print(cmd1)
        print(type(cmd1))
        sendProgram(self.robot, motion_program)
        res.success = True
        return res


def main():
    rclpy.init()
    api = ROSAPI()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)


if __name__ == '__main__':
    main()
