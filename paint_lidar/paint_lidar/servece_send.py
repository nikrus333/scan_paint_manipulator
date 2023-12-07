import sys
import math
from scipy.spatial.transform import Rotation as R
import numpy as np
from threading import Thread
import asyncio
import time


import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, Quaternion, Pose, Point
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster, Buffer, StaticTransformBroadcaster
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseArray
from rclpy.action import ActionClient
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener

from example_interfaces.action import ExecuteTrajectoryArray
from example_interfaces.srv import SetBool
from example_interfaces.srv import SetAngle
from example_interfaces.srv import PoseTf

from .lidar_utils import test_driver_laser, trajectory
from .lidar_utils.enum_set import ModeWork, ChooseStartManip, ParametrsManipulator, SelectModeWork
from .lidar_utils.trajectory import euler_from_quaternion


class MinimalClientAsync(Node):

    def __init__(self):
        
        super().__init__('minimal_client_async')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_buffer_1 = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_listener1 = TransformListener(self.tf_buffer_1, self, spin_thread=True)
        self.cli = self.create_client(SetBool, '/point')
        self.cli_tf_send = self.create_client(PoseTf, '/service_send_tf')
        self.cli_pose_eig = self.create_client(SetAngle, '/arrow_points_service')
        self.sub_angle = self.create_subscription(Float32MultiArray, "/angle_points", self.callbackAngle, qos_profile=10)
        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, ExecuteTrajectoryArray, 'execute_trajectory', callback_group=self.callback_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service  POINt not available')
        while not self.cli_pose_eig.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service  ARROW POINTS SERVICE not available')
        # while not self.cli_tf_send.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service  TF SEND SERVICE not available')
        self.req = SetBool.Request()

        self.points = None
        self.angle = None
        self.rotation = None
        self.trans = None
        self.count_paint_make = 0

    def send_request(self, mode_work):
        match mode_work:
            case ModeWork.SCAN_AND_PAINT.value:
                self.req.mode = 'scan'
            case ModeWork.CALCULATE_RANGE.value:
                self.req.mode = 'calculate_range'  # 'scan'
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_request_angle(self, tf_pose, step):
        self.req_angle = SetAngle.Request()
        pose = Pose()
        
        pose.position.x = tf_pose.transform.translation.x
        pose.position.y = tf_pose.transform.translation.y
        pose.position.z = tf_pose.transform.translation.z
        pose.orientation = tf_pose.transform.rotation
        self.req_angle.pose = pose
        self.req_angle.step = step
        print(pose)
        self.future_angle = self.cli_pose_eig.call_async(self.req_angle)
        rclpy.spin_until_future_complete(self, self.future_angle)
        return self.future_angle.result()
 
    def send_request_tf(self, tf: TransformStamped):
        req_tf_send = PoseTf.Request()
        req_tf_send.pose.position.x = tf.transform.translation.x
        req_tf_send.pose.position.y = tf.transform.translation.y
        req_tf_send.pose.position.z = tf.transform.translation.z
        
        req_tf_send.pose.orientation.x = tf.transform.rotation.x
        req_tf_send.pose.orientation.y = tf.transform.rotation.y
        req_tf_send.pose.orientation.z = tf.transform.rotation.z
        req_tf_send.pose.orientation.w = tf.transform.rotation.w
        
        req_tf_send.child_name = tf.child_frame_id
        req_tf_send.parent_name = tf.header.frame_id
        future = self.cli_tf_send.call_async(req_tf_send)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def listener_tf_trajectory(self, parent: str, child: str) -> TransformStamped:
        try:
            transform_stamped = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            return transform_stamped
        except TransformException as ex:    
            self.get_logger().info(f'Could not transform {parent} to {child}: {ex}') 
            return False
    
    def updateTF(self):
        for i in range(10):
            rclpy.spin_once(self)
            time.sleep(.01)
        pass
   
    def listener_tf(self):
        for i in range(5):
            try:
                t = self.tf_buffer_1.lookup_transform('world', 'manipulator', rclpy.time.Time(),  timeout=rclpy.duration.Duration(seconds=1.0))
                return t
            except TransformException as ex:    
                self.get_logger().info(f'Could not transform world to manipulator: {ex}') 
        return False
        

    def tf_call_tool(self, array_slice, euler=None): # local to global
        global_array_slice = []
        # self.tf_broadcaster = StaticTransformBroadcaster(self)
        tf_world_manipulator = self.listener_tf_trajectory("world", "manipulator")
        roll, pitch, yaw = trajectory.euler_from_quaternion([tf_world_manipulator.transform.rotation.x, tf_world_manipulator.transform.rotation.y, tf_world_manipulator.transform.rotation.z, tf_world_manipulator.transform.rotation.w])
        pi = math.pi
        now = self.get_clock().now()
        for count_line, slice in enumerate(array_slice):
            global_points = []
            for count, point in enumerate(slice):
                transform_stamped_msg2 = TransformStamped()
                transform_stamped_msg2.header.stamp = self.get_clock().now().to_msg()
                transform_stamped_msg2.header.frame_id = 'world'
                transform_stamped_msg2.child_frame_id = str(
                    count_line) + "_" + str(count)
                r1 = R.from_matrix(euler)
                x, y, z, w = r1.as_quat()
                point = trajectory.transform_point(point, [.0, .0, .0], [roll, pitch, yaw])
                # + center[0]
                transform_stamped_msg2.transform.translation.x = tf_world_manipulator.transform.translation.x + point[0]
                # + center[1]
                transform_stamped_msg2.transform.translation.y = tf_world_manipulator.transform.translation.y + point[1]
                # + center[2]
                transform_stamped_msg2.transform.translation.z = tf_world_manipulator.transform.translation.z + point[2]
                # normale[0]#quaternion.x
                new_quat = test_driver_laser.PaintScanWall.multiplay_quaternion(tf_world_manipulator.transform.rotation, Quaternion(x=x, y=y, z=z, w=w))
                transform_stamped_msg2.transform.rotation.x = new_quat.x
                # normale[1]#quaternion.y
                transform_stamped_msg2.transform.rotation.y = new_quat.y
                # normale[2]#quaternion.z
                transform_stamped_msg2.transform.rotation.z = new_quat.z
                # normale[3]#quaternion.w
                transform_stamped_msg2.transform.rotation.w = new_quat.w
                global_points.append(transform_stamped_msg2)
                # self.send_request_tf(transform_stamped_msg2)
                self.tf_broadcaster.sendTransform(transform_stamped_msg2)
                self.tf_buffer.set_transform_static(transform_stamped_msg2, str(count_line) + "_" + str(count))
            global_array_slice.append(np.array(global_points))
        return global_array_slice
    
    def tf_call_tool_rotate(self, tf_last, step, rotation, count_plane, x_step=0.03):
        # tf_broadcaster = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        leng = int(step / x_step)
        for count_line, tf in enumerate(tf_last):
            ts = TransformStamped()
            ts.header.frame_id = "world"
            ts.header.stamp = now
            ts.child_frame_id = "tf" + str(count_plane) + str(count_line)
            ts.transform.translation = tf.transform.translation
            ts.transform.rotation = rotation
            self.tf_buffer.set_transform_static(ts, "tf" + str(count_plane) + str(count_line))
            self.tf_broadcaster.sendTransform(ts)
            # self.send_request_tf(ts)
            for i in range(leng + 1):
                if count_line % 2 == 0:
                    count = leng - i
                else:
                    count = i
                transform_stamped_msg = TransformStamped()
                transform_stamped_msg.header.stamp = now
                transform_stamped_msg.header.frame_id = "tf" + str(count_plane) + str(count_line)
                transform_stamped_msg.child_frame_id = str(count_plane) + '_' + str(
                    count_line) + "_" + str(count)
                transform_stamped_msg.transform.rotation.x = 0.0
                transform_stamped_msg.transform.rotation.y = 0.0
                transform_stamped_msg.transform.rotation.z = 0.0
                transform_stamped_msg.transform.rotation.w = 1.0
                transform_stamped_msg.transform.translation.x = -x_step * i
                transform_stamped_msg.transform.translation.y = .0
                transform_stamped_msg.transform.translation.z = .0
                self.tf_broadcaster.sendTransform(transform_stamped_msg)
                self.tf_buffer.set_transform_static(transform_stamped_msg, str(count_plane) + '_' + str(
                    count_line) + "_" + str(count))
                # self.send_request_tf(transform_stamped_msg)
    
    def callbackAngle(self, data: Float32MultiArray):
        self.angle = data.data

    def new_trajectory(self, array_slice):
        new_trajectory = []
        for count_line, slice in enumerate(array_slice):
            new_points = []
            for count, point in enumerate(slice):
                new_points.append(trajectory.transform_point([.0, .0, .0], [point[0], point[1], point[2]], [.0, .0, -self.angle[0]]))
            new_trajectory.append(new_points)
        return new_trajectory

    def trajectory_slice(self, array_slice, name):
        new_trajectory = []
        tf_world_manipulator = self.listener_tf_trajectory('world', 'manipulator')
        for count_line, slice in enumerate(array_slice):
            new_points = []
            for count, point in enumerate(slice):
                tf = self.listener_tf_trajectory('world', name[0] + str(count_line) + "_" + str(count))
                if tf is not None:
                    new_tf = TransformStamped()
                    new_tf.header.stamp = self.get_clock().now().to_msg()
                    new_tf.header.frame_id = ""
                    x = tf.transform.translation.x - tf_world_manipulator.transform.translation.x
                    y = tf.transform.translation.y - tf_world_manipulator.transform.translation.y
                    z = tf.transform.translation.z - tf_world_manipulator.transform.translation.z
                    np.append(new_points, [x, y, z])
                else:
                    break
            np.append(new_trajectory, new_points)
        return new_trajectory

    
    def get_last_point(self, array_slice, count_paint_make, name = "right"):
        width = len(array_slice[0])
        height = len(array_slice)
        if name == "right":
            if count_paint_make == 1:
                num = [f"{i}_{0 if i % 2 == 0 else width-1}" for i in range(height)]
            else:
                num = [f"{count_paint_make-1}_{i}_{0 if i % 2 == 0 else width-1}" for i in range(height)]
        elif name == "left":
            if count_paint_make == 1:
                num = [f"{i}_{width-1 if i % 2 == 0 else 0}" for i in range(height)]
            else:
                num = [f"{count_paint_make-1}_{i}_{width-1 if i % 2 == 0 else 0}" for i in range(height)]
        else:
            num = None
        print(num)
        if num != None:
            last_points = []
            # Take last points from trajectory
            for i, item in enumerate(num):
                last_points.append(self.listener_tf_trajectory('world', item))
                pass
            return last_points
        else:
            return None

    def send_goal(self, array_traject_msg):
        goal_msg = ExecuteTrajectoryArray.Goal()
        pi = math.pi
        print(array_traject_msg[0])
        for traject in array_traject_msg:
            goal_msg.poses_array.append(traject)

        goal_msg.acceleration = 0.02
        goal_msg.velocity = 0.08
        goal_msg.type_traject = 'color'
        # print(goal_msg)
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_goal_sonya_point(self, order):
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
        pose1.position.x = 0.65
        pose1.position.y = 0.38
        pose1.position.z = 0.7
        x, y, z, w = r1.as_quat()

        pose1.orientation.x = math.radians(90)
        pose1.orientation.y = math.radians(0)
        pose1.orientation.z = math.radians(90)
        pose1.orientation.w = w

        # pose2.position.x = 47/1000
        # pose2.position.y = -592/1000
        # pose2.position.z = 291/1000

        pose2.position.x = 0.65
        pose2.position.y = -0.39
        pose2.position.z = 0.7

        pose2.orientation.x = math.radians(90)
        pose2.orientation.y = math.radians(0)
        pose2.orientation.z = math.radians(90)
        pose2.orientation.w = w
        pose_arr1.poses = [pose1, pose2]

        goal_msg.poses_array = [pose_arr1]

        goal_msg.acceleration = 0.02
        goal_msg.velocity = 0.008
        goal_msg.type_traject = 'scan'
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    # вызывается при вызове сервиса
    # старт манипулятора и старт сканирования

    def feedback_callback(self, feedback_msg):
        pass
        # value = feedback_msg.feedback.robot_state
        # self.get_logger().info('feed : {0}'.format(value))
        # #params = self.robot.subscription.read()
        # #print(params
        # if value == [0, 0, 0, 0, 0, 0]:
        #     return False
        # trans, euler = value[:3], value[3:]

        # trans_init, R = self.hok.coord_euler_to_matrix(trans, euler)

        # #print(trans_init)

        # self.pcd = self.pcd + self.hok.read_laser(trans_init, R)

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

    def create_msg(self, array_slise, euler):
        pi = math.pi
        array_traject_msg = []
        for slice in array_slise:
            pose_array = PoseArray()
            for point in slice:
                pose1 = Pose()
                pose1.position.x = point[0]
                pose1.position.y = point[1]
                pose1.position.z = point[2]
                r1 = R.from_euler('zyx', euler)
                x, y, z, w = r1.as_quat()
                pose1.orientation.x = euler[0]
                pose1.orientation.y = euler[1]
                pose1.orientation.z = euler[2]
                pose1.orientation.w = w
                pose_array.poses.append(pose1)
            array_traject_msg.append(pose_array)
        return array_traject_msg

    def tf_eig(self, rot, trans):
        # tf_broadcaster = StaticTransformBroadcaster(self)
        now = self.get_clock().now()
        transform_stamped_msg2 = TransformStamped()
        transform_stamped_msg2.header.stamp = now.to_msg()
        transform_stamped_msg2.header.frame_id = 'manipulator'
        transform_stamped_msg2.child_frame_id = 'eig'

        tfqt = Quaternion()

        r = R.from_matrix(rot)
        q = r.as_quat()
        # + center[0]
        transform_stamped_msg2.transform.translation.x = trans[0]
        # + center[1]
        transform_stamped_msg2.transform.translation.y = trans[1]
        # + center[2]
        transform_stamped_msg2.transform.translation.z = trans[2]
        transform_stamped_msg2.transform.rotation.x = q[0]  # quaternion.x
        transform_stamped_msg2.transform.rotation.y = q[1]  # quaternion.y
        transform_stamped_msg2.transform.rotation.z = q[2]  # quaternion.z
        transform_stamped_msg2.transform.rotation.w = q[3]  # quaternion.w
        self.tf_broadcaster.sendTransform(transform_stamped_msg2)
        return transform_stamped_msg2

    def tf_eig_world(self, tf):
        # tf_broadcaster = StaticTransformBroadcaster(self)
        now = self.get_clock().now()
        transform_stamped_msg2 = tf
        transform_stamped_msg2.header.stamp = now.to_msg()
        
        self.tf_broadcaster.sendTransform(transform_stamped_msg2)
        
        return transform_stamped_msg2
    
    def tf_listener_traject_manipulator(self):
        array_traject_msg = []
        slices = 3
        name_point = str(self.count_paint_make)
        for slic in range(slices):
            count_point = 0
            new_trajectory = PoseArray()
            while True:
                pose = Pose()
                # print('______________________________________')
                # print('manipulator', name_point + '_' + str(slic) + '_' + str(count_point))
                tf_manipulator_point = self.listener_tf_trajectory('manipulator', name_point + '_' + str(slic) + '_' + str(count_point))   
                # print(tf_manipulator_point)
                count_point +=1 
                if tf_manipulator_point is False:
                    break
                else:
                    pose.position.x = tf_manipulator_point.transform.translation.x
                    pose.position.y = tf_manipulator_point.transform.translation.y
                    pose.position.z = tf_manipulator_point.transform.translation.z
                    q = [tf_manipulator_point.transform.rotation.x, tf_manipulator_point.transform.rotation.y,
                         tf_manipulator_point.transform.rotation.z, tf_manipulator_point.transform.rotation.w]
                    x, y, z = euler_from_quaternion(q)
                    pose.orientation.x = z
                    pose.orientation.y = y
                    pose.orientation.z = x
                    pose.orientation.w = tf_manipulator_point.transform.rotation.w
           
        # tb = StaticTransformBroadcaster(self)          pose.orientation.z = z
                    new_trajectory.poses.append(pose)
            array_traject_msg.append(new_trajectory)   
            
        tf = TransformStamped()
        tf.header.frame_id = "manipulator"
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.child_frame_id = "orient"
        tf.transform.translation.x = array_traject_msg[0].poses[0].position.x
        tf.transform.translation.y = array_traject_msg[0].poses[0].position.y
        tf.transform.translation.z = array_traject_msg[0].poses[0].position.z
        tf.transform.rotation = array_traject_msg[0].poses[0].orientation
        self.tf_broadcaster.sendTransform(tf)
        return array_traject_msg

    
    def chose_mode_work(self) -> int:
        while True:
            print('Введите номер режима работы')
            print('1 - сканировние и окраска')
            print('2 - расстояние до поверхности')
            print('3 - окраска, без сканирования')
            mode_work = int(input())
            if (mode_work == ModeWork.SCAN_AND_PAINT.value) or (mode_work == ModeWork.CALCULATE_RANGE.value) or (mode_work == ModeWork.ONLY_PAINT.value):
                break
            else:
                print('Некорректный ввод')
        return mode_work

    def chose_start_manip(self) -> int:
        while True:
            print('Проверьте точки траектории')
            print('Точки траектории корректны')
            print('1 - Да')
            print('2 - Нет')
            chose_start_manip = int(input())
            if (chose_start_manip == ChooseStartManip.SKIP_TRAJECTORY.value) or (chose_start_manip == ChooseStartManip.START_MANIPULATOR.value):
                return chose_start_manip
            else:
                print('Некорректный ввод')


def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    paint = test_driver_laser.PaintScanWall()
    minimal_client.count_paint_make = 0
    flag_paint = False
    while True:
        print('______________________________________')
        minimal_client.updateTF()
        mode_work = minimal_client.chose_mode_work()
        match mode_work:
            case ModeWork.SCAN_AND_PAINT.value:
                response = minimal_client.send_request(mode_work)
                x_data, y_data, z_data = response.x_data, response.y_data, response.z_data
                numpy_arr = paint.convert_srv_np(x_data, y_data, z_data)
                pcd_new = paint.NumpyToPCD(numpy_arr)
                pcd_new = pcd_new.transform(np.array([[1, 0, 0, ParametrsManipulator.TRANS_PLANE.value - ParametrsManipulator.DIST_PAINT.value],
                                                        [0, 1, 0, 0],
                                                        [0, 0, 1, 0],
                                                        [0, 0, 0, 1]]))

                plane_list = paint.DetectMultiPlanes(
                    pcd_new, min_ratio=0.15, threshold=0.017, iterations=1000)
                
                if  SelectModeWork.VISUALIZATION.value:
                    paint.DrawPlanes(plane_list)

                if len(plane_list) > 1:
                    plane_list = paint.select_plane(plane_list)
                else:
                    print('Обноружена одна поверхность')
                
                if minimal_client.count_paint_make == 0:    
                    array_slice, euler, rotation, trans, dist_point_to_plane, step = paint.CreateTraectory_circle_vertical(
                        plane_list)
                    print('Дистанция до поверхности : ', dist_point_to_plane)
                    if (dist_point_to_plane < ParametrsManipulator.MIN_DISTATION.value
                            or dist_point_to_plane > ParametrsManipulator.MAX_DISTATION.value):
                        print('Дистанция слишком мала/велика')
                        print('Передвиньте стрелу')
                    if True:
                            euler1 = paint.rot2euler(rotation)
                            array_traject_msg = minimal_client.create_msg(
                                array_slice, euler1)
                            # print('array_traject_msg  ', array_traject_msg)
                            # print('euler ', euler)
                            minimal_client.rotation, minimal_client.trans = rotation, trans
                            transform_manipulator_eig = minimal_client.tf_eig(rotation, trans)
                            global_tf_slice = minimal_client.tf_call_tool(array_slice=array_slice, euler=rotation)
                            transform_world_manipulator = minimal_client.listener_tf()
                            transform_eig_world = test_driver_laser.PaintScanWall.sum_tf_stemp(transform_world_manipulator, transform_manipulator_eig, 
                                                                                            minimal_client.get_clock().now().to_msg())
                            minimal_client.tf_eig_world(transform_eig_world)
                            response = minimal_client.send_request_angle(transform_eig_world, step)
                            flag_paint = True
                else:
                    minimal_client.updateTF()
                    transform_manipulator_eig = minimal_client.tf_eig(rotation, trans)
                    transform_world_manipulator = minimal_client.listener_tf()
                    transform_eig_world = test_driver_laser.PaintScanWall.sum_tf_stemp(transform_world_manipulator, transform_manipulator_eig, 
                                                                                    minimal_client.get_clock().now().to_msg())
                    _0, _01, rotation, _02, _03, _04 = paint.CreateTraectory_circle_vertical(
                        plane_list)

                            # chose_start_manip = minimal_client.chose_start_manip()
                            # match chose_start_manip:
                            #     case ChooseStartManip.START_MANIPULATOR.value:
                            #         minimal_client.send_goal(array_traject_msg)
                            #         count_paint_make +=1
                            #     case ChooseStartManip.SKIP_TRAJECTORY.value:
                            #         pass
            case ModeWork.CALCULATE_RANGE.value:
                response = minimal_client.send_request(mode_work)
                pass

            case ModeWork.ONLY_PAINT.value:
                print('Дистанция до поверхности : ', dist_point_to_plane)
                if (dist_point_to_plane < ParametrsManipulator.MIN_DISTATION.value
                        or dist_point_to_plane > ParametrsManipulator.MAX_DISTATION.value):
                    print('Дистанция слишком мала/велика')
                    print('Передвиньте стрелу')
                if True:
                    #####print(minimal_client.tf_listener_traject_manipulator())
                    if minimal_client.count_paint_make == 0:
                        # transform_manipulator_eig = minimal_client.tf_eig(rotation, trans)
                        global_tf_slice = minimal_client.tf_call_tool(array_slice=array_slice, euler=rotation)
                        # transform_world_manipulator = minimal_client.listener_tf()
                        # transform_eig_world = test_driver_laser.PaintScanWall.sum_tf_stemp(transform_world_manipulator, transform_manipulator_eig, 
                        #                                                                    minimal_client.get_clock().now().to_msg())
                        # minimal_client.tf_eig_world(transform_eig_world)
                        response = minimal_client.send_request_angle(transform_eig_world, step)
                        chose_start_manip = minimal_client.chose_start_manip()
                        match chose_start_manip:
                            case ChooseStartManip.START_MANIPULATOR.value:
                                minimal_client.send_goal(array_traject_msg)
                                minimal_client.count_paint_make +=1
                                flag_paint = False
                            case ChooseStartManip.SKIP_TRAJECTORY.value:
                                pass   
                    else:
                        minimal_client.updateTF()
                        tf_last = minimal_client.get_last_point(array_slice, minimal_client.count_paint_make)
                        # if flag_paint:
                        # minimal_client.rotation, minimal_client.trans = rotation, trans
                        # transform_manipulator_eig = minimal_client.tf_eig(rotation, trans)
                        # transform_world_manipulator = minimal_client.listener_tf()
                        # transform_eig_world = test_driver_laser.PaintScanWall.sum_tf_stemp(transform_world_manipulator, transform_manipulator_eig, 
                        #                                                                 minimal_client.get_clock().now().to_msg())
                        # minimal_client.tf_eig_world(transform_eig_world)
                        # print("tfmanip", transform_world_manipulator)
                        transform_world_manipulator = minimal_client.listener_tf()
                        minimal_client.tf_call_tool_rotate(tf_last, step, transform_eig_world.transform.rotation, minimal_client.count_paint_make)
                        array_traject_msg = minimal_client.tf_listener_traject_manipulator()
                        chose_start_manip = minimal_client.chose_start_manip()
                        print('______________array_____________________')
                        print(array_traject_msg)
                        match chose_start_manip:
                            case ChooseStartManip.START_MANIPULATOR.value:
                                minimal_client.send_goal(array_traject_msg)
                                minimal_client.count_paint_make +=1
                                response = minimal_client.send_request_angle(transform_eig_world, step)
                            case ChooseStartManip.SKIP_TRAJECTORY.value:
                                pass 
        print('______________________________________')
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
