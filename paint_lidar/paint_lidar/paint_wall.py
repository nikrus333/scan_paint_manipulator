import time
from threading import Event

from example_interfaces.srv import SetPC2
from example_interfaces.action import ExecuteTrajectoryArray


import motorcortex
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.action import ActionClient
from action_tutorials_interfaces.action import Fibonacci
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseArray


import open3d as o3d
import numpy as np
import os
import random
import math
from threading import Event
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R

from .lidar_utils import test_driver_laser
from .lidar_utils import servoPy
from .lidar_utils.enum_set import SelectModeWork
from .lidar_utils.trajectory import euler_from_quaternion, quaternion_from_euler

class ServiceFromService(Node):

    def __init__(self):
        super().__init__('action_from_service')

        # self.robot = ManipUse()
        self.service_done_event = Event()

        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, ExecuteTrajectoryArray, 'execute_trajectory', callback_group=self.callback_group)

        self.srv = self.create_service(
            SetPC2,
            '/point',
            self.add_two_ints_callback,
            callback_group=self.callback_group
        )
        # self.create_timer(1, self.feedback_callback, callback_group=self.callback_group)

        self.status_manipulator = False
        self.scan_wall_flag = True

        try:
            self.hok = test_driver_laser.HokuyoManipulator()
        except (IOError, EOFError) as e:
            print("Невозможно открыть USB порт lidara  {}".format(e.args[-1]))
        try:
            self.servo = servoPy.closeOpen(simulation=SelectModeWork.SERVO_LIDAR_SIMULATION.value)
        except (IOError, EOFError) as e:
            print(
                "Невозможно открыть USB порт системы защиты lidara {}".format(e.args[-1]))

        self.pcd = o3d.geometry.PointCloud()
    # *optionally* add initial points
        self.points = []
        self.points.append([0.0, 0.0, 1.0])
        self.points = np.array(self.points)
        self.paint = test_driver_laser.PaintScanWall()

    def add_two_ints_callback(self, request: SetPC2.Request, response: SetPC2.Response) -> SetPC2.Response:
        debug = SelectModeWork.MANIPULATOR_SIMULATION.value
        mode = request.mode
        message = ' Сообщение Request received: мод работы ' + mode
        self.get_logger().info(message)

        event = Event()

        def done_callback(future):
            nonlocal event
            event.set()

        if mode == 'scan':
            pcd_list = []
            print('mode scan start')
            pose_list = self.getPoints()
            for i, pose_array in enumerate(pose_list):
                print("step:", i)
                self.status_manipulator = False
                self.scan_wall_flag = True
                # response = scan_wall(response=response, debug=debug)
                debug = SelectModeWork.MANIPULATOR_SIMULATION.value
                if not debug:
                    future = self.send_goal(pose_array)
                    # future = self.send_goal_rotate(10)
                    if not SelectModeWork.SERVO_LIDAR_SIMULATION.value:
                        self.servo.open()
                # self.robot.mission_scan()
                try:
                    if not debug:
                        while not self.status_manipulator:
                            previous_t = time.time()

                except KeyboardInterrupt:
                    print('End process scan')
                finally:
                    if not SelectModeWork.SERVO_LIDAR_SIMULATION.value:
                        self.servo.close()
                    pcd = self.pcd
                    #o3d.io.write_point_cloud('src/paint_river_volga/paint_lidar/scan_obj/scan_23_08.pcd', pcd) # save pcd data
                    if SelectModeWork.LIDAR_SIMULATION.value:
                        pcd_new = o3d.io.read_point_cloud("/home/alex/ros2_scan/src/scan_paint_manipulator/paint_lidar/scan_obj/scan_23_08.pcd")
                    else:
                        pcd_new = pcd

                    if SelectModeWork.VISUALIZATION.value:
                        o3d.visualization.draw_geometries([pcd_new])
                    
                    pcd_list.append(pcd_new)
                    del pcd, self.pcd
                    self.pcd = o3d.geometry.PointCloud()
                    print("End")
            
            print("here")
            for i, pcd_ in enumerate(pcd_list):
                print(i)
                pc2 = self.paint.PCDToPC2(self, pcd_, "base_link")
                response.pc2.append(pc2)
            
            print("success!")
            response.success = True
            #  numpy_arr = self.paint.PCDToNumpy(pcd_new)
            # response.x_data, response.y_data, response.z_data = self.paint.convert_np_srv(numpy_arr)
            # *optionally* add initial points
            self.points = []
            self.points.append([0.0, 0.0, 1.0])
            self.points = np.array(self.points)
            self.status_manipulator = False
            return response
            
        if mode == 'calculate_range':
            self.scan_wall_flag = False
            if not debug:
                future = self.send_goal(10)
                # future = self.send_goal_rotate(10)
                if not SelectModeWork.SERVO_LIDAR_SIMULATION.value:
                    self.servo.open()
            try:
                if not debug:
                    while not self.status_manipulator:
                        pass
                        # print('расстояние ')
            except KeyboardInterrupt:
                print('End process scan')
            finally:
                print('Остановка процесса измерения расстояния')
                if not SelectModeWork.SERVO_LIDAR_SIMULATION.value:
                    self.servo.close()
                self.status_manipulator = False
                response.success = True
                return response
        return response

    def getPoints(self) -> list[PoseArray]:
        """Create array pose for N scanning points"""
        point_list = []
        for i in range(3):
            line = PoseArray()
            pose1 = Pose()
            pose1.position.x = 0.640
            pose1.position.y = -0.450 + i*0.450
            pose1.position.z = 0.4
            q = quaternion_from_euler(math.radians(180), math.radians(0), math.radians(90))
            pose1.orientation.x = q[0]
            pose1.orientation.y = q[1]
            pose1.orientation.z = q[2]
            pose1.orientation.w = q[3]
            line.poses.append(pose1)

            pose2 = Pose()
            pose2.position.x = 0.540
            pose2.position.y = -0.450 + i*0.450
            pose2.position.z = 0.4
            pose2.orientation = pose1.orientation
            line.poses.append(pose2)
            point_list.append(line)
        return point_list

    def send_goal_rotate(self, order):
        pi = math.pi
        goal_msg = ExecuteTrajectoryArray.Goal()
        poses = []
        pose_arr1 = PoseArray()
        pose1 = Pose()
        pose2 = Pose()

        # pose1.position.x = 47/1000
        # pose1.position.y = -406/1000
        # pose1.position.z = 289/1000
        # r1 = R.from_euler('zyx', [0 * pi / 180, 0 * pi / 180, 180 * pi / 180])
        r1 = R.from_euler('zyx', [math.radians(
            90), math.radians(0), math.radians(90)])
        pose1.position.x = 0.640
        pose1.position.y = -0.039
        pose1.position.z = 0.6
        x, y, z, w = r1.as_quat()

        pose1.orientation.x = math.radians(90)
        pose1.orientation.y = math.radians(0)
        pose1.orientation.z = math.radians(120)
        pose1.orientation.w = w

        # pose2.position.x = 47/1000
        # pose2.position.y = -592/1000
        # pose2.position.z = 291/1000

        pose2.position.x = 0.640
        pose2.position.y = -0.039
        pose2.position.z = 0.6

        pose2.orientation.x = math.radians(90)
        pose2.orientation.y = math.radians(0)
        pose2.orientation.z = math.radians(60)
        pose2.orientation.w = w
        pose_arr1.poses = [pose1, pose2]

        goal_msg.poses_array = [pose_arr1]

        goal_msg.acceleration = 0.02
        goal_msg.velocity = 0.008
        goal_msg.rotational_velocity = 0.0318
        goal_msg.rotational_acceleration = 0.0637
        goal_msg.type_traject = 'scan'
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_goal(self, pose_array: PoseArray):
        pi = math.pi
        goal_msg = ExecuteTrajectoryArray.Goal()
        goal_msg.poses_array = [pose_array]

        goal_msg.acceleration = 0.02
        goal_msg.velocity = 0.008
        goal_msg.rotational_velocity = 0.318
        goal_msg.rotational_acceleration = 0.637
        goal_msg.type_traject = 'scan'
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_goal_up(self, order):
        pi = math.pi
        goal_msg = ExecuteTrajectoryArray.Goal()
        poses = []
        pose_arr1 = PoseArray()
        pose1 = Pose()
        pose2 = Pose()

        # pose1.position.x = 47/1000
        # pose1.position.y = -406/1000
        # pose1.position.z = 289/1000
        # r1 = R.from_euler('zyx', [0 * pi / 180, 0 * pi / 180, 180 * pi / 180])
        r1 = R.from_euler('zyx', [math.radians(
            90), math.radians(0), math.radians(90)])
        pose1.position.x = -0.445
        pose1.position.y = -0.280
        pose1.position.z = 0.77
        x, y, z, w = r1.as_quat()
        pose1.orientation.x = math.radians(-90)
        pose1.orientation.y = math.radians(0)
        pose1.orientation.z = math.radians(0)
        pose1.orientation.w = w

        # pose2.position.x = 47/1000
        # pose2.position.y = -592/1000
        # pose2.position.z = 291/1000

        pose2.position.x = -0.354
        pose2.position.y = -0.280
        pose2.position.z = 0.77
        pose2.orientation.x = math.radians(-90)
        pose2.orientation.y = math.radians(0)
        pose2.orientation.z = math.radians(0)
        pose2.orientation.w = w

        pose_arr1.poses = [pose1, pose2]

        goal_msg.poses_array = [pose_arr1]

        goal_msg.acceleration = 0.02
        goal_msg.velocity = 0.008
        goal_msg.rotational_velocity = 0.318
        goal_msg.rotational_acceleration = 0.637
        goal_msg.type_traject = 'scan'
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    # вызывается при вызове сервиса
    # старт манипулятора и старт сканирования

    def feedback_callback(self, feedback_msg):
        if self.scan_wall_flag:
            print('scan wall flag true')
            value = feedback_msg.feedback.robot_state
            self.get_logger().info('feed : {0}'.format(value))
            if value == [0, 0, 0, 0, 0, 0]:
                return False
            trans, euler = value[:3], value[3:]
            if not SelectModeWork.LIDAR_SIMULATION.value:
                trans_init, R = self.hok.coord_euler_to_matrix(trans, euler)
                self.pcd = self.pcd + self.hok.read_laser(trans_init, R)
            else:
                self.pcd = self.pcd
        if self.scan_wall_flag == False:
            value = feedback_msg.feedback.robot_state
            if value == [0, 0, 0, 0, 0, 0]:
                return False
            trans, euler = value[:3], value[3:]
            if not SelectModeWork.LIDAR_SIMULATION.value:
                trans_init, R = self.hok.coord_euler_to_matrix(trans, euler)
                dist_x = self.hok.read_laser_scan_dist(trans_init, R)
                if dist_x > 0:
                    print('К стенене на : ', dist_x)
                else:
                    print('От стены на :', dist_x)
            else:
                print('К стенене на : -')
            

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        print('hereee')
        print(future)
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.status_manipulator = result.success


def main(args=None):
    # manip = ManipUse()
    rclpy.init(args=args)

    service_from_service = ServiceFromService()

    executor = MultiThreadedExecutor()
    executor.add_node(service_from_service)
    executor.spin()
    # rclpy.spin(service_from_service, executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
