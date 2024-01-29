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
from example_interfaces.srv import SetPC2, SetAngle, PoseTf, TrajectoryMode
from example_interfaces.msg import CommandPose, TrajectoryMsg, PoseMsg

from .lidar_utils import test_driver_laser, trajectory
from .lidar_utils.enum_set import ModeWork, ChooseStartManip, ParametrsManipulator, SelectModeWork
from .lidar_utils.trajectory import euler_from_quaternion
from .lidar_utils.gcode import Gcode, MOVC, MOVL


class ServiceTrajectory(Node):

    def __init__(self):
        super().__init__('service_trajectory')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_buffer_1 = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_listener1 = TransformListener(self.tf_buffer_1, self, spin_thread=True)
        self.callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(TrajectoryMode, '/service_trajectory', self.status_callback, callback_group=self.callback_group)
        self.cli = self.create_client(SetPC2, '/point', callback_group=self.callback_group)
        self.cli_pose_eig = self.create_client(SetAngle, '/arrow_points_service', callback_group=self.callback_group)
        self._action_client = ActionClient(self, ExecuteTrajectoryArray, 'execute_trajectory', callback_group=self.callback_group)
        attempted = 0
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action execute not available')
            attempted += 1
            if attempted >= 5:
                print("Couldn`t to connect to execute_trajectory")
                break

        # print("Yes points service")
        # while not self.cli_pose_eig.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service  ARROW POINTS SERVICE not available')
        # print("Yes arrow service")

        self.points = None
        self.angle = None
        self.rotation = None
        self.trans = None
        self.dist_point_to_plane = None
        self.array_slice = None
        self.transform_eig_world = None
        self.step = None
        self.mode = False
        self.offset = Point(x=.0, y=.0, z=.0)

        self.count_paint_make = 0

    def send_request(self, mode_work: str) -> SetPC2.Response:
        req = SetPC2.Request()
        match mode_work:
            case ModeWork.SCAN_AND_PAINT.value:
                req.mode = 'scan'
            case ModeWork.CALCULATE_RANGE.value:
                req.mode = 'calculate_range'  # 'scan'
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)       
        return self.future.result()

    def send_request_angle(self, tf_pose: TransformStamped, step: float):
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
    
    def listener_tf_trajectory(self, parent: str, child: str) -> TransformStamped:
        for i in range(5):
            try:
                transform_stamped = self.tf_buffer.lookup_transform(parent, child, Time(), timeout=rclpy.duration.Duration(seconds=0.1))
                return transform_stamped
            except TransformException as ex:    
                self.get_logger().info(f'Could not transform {parent} to {child}: {ex}') 
        return None
   
    def listener_tf(self):
        for i in range(5):
            try:
                t = self.tf_buffer_1.lookup_transform('world', 'manipulator', rclpy.time.Time(),  timeout=rclpy.duration.Duration(seconds=1.0))
                return t
            except TransformException as ex:    
                self.get_logger().info(f'Could not transform world to manipulator: {ex}') 
        return None
        
    def tf_call_tool(self, array_slice, euler=None): # local to global
        global_array_slice = []
        tf_world_manipulator = self.listener_tf_trajectory("world", "manipulator")
        roll, pitch, yaw = trajectory.euler_from_quaternion([tf_world_manipulator.transform.rotation.x, tf_world_manipulator.transform.rotation.y, tf_world_manipulator.transform.rotation.z, tf_world_manipulator.transform.rotation.w])
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
    
    def createTFPoints(self, list_tf:list[TransformStamped], count_plane: int, rotation):
        for tf in list_tf:
            tf_name = f"tf{tf.child_frame_id}"
            ts = TransformStamped()
            ts.header.frame_id = "world"
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.child_frame_id = tf_name
            ts.transform.translation = tf.transform.translation
            ts.transform.rotation = rotation
            self.tf_buffer.set_transform_static(ts, tf_name)
            self.tf_broadcaster.sendTransform(ts)
    
    def tf_call_tool_rotate(self, tf_last: list[TransformStamped], step: float, rotation: TransformStamped, count_plane: int, name: str, x_step=0.03, y_step=0.3):
        mode = -1 if name == "left" else 1
        now = self.get_clock().now().to_msg()
        print("name:", name)
        for tf in tf_last:
            tf_name = f"tf{tf.child_frame_id}"
            ts = TransformStamped()
            ts.header.frame_id = "world"
            ts.header.stamp = now
            ts.child_frame_id = tf_name
            ts.transform.translation = tf.transform.translation
            ts.transform.rotation = rotation
            self.tf_buffer.set_transform_static(ts, tf_name)
            self.tf_broadcaster.sendTransform(ts)

        if name == "right" or name == "left":
            leng = int(step / x_step)
            for count_line, tf in enumerate(tf_last):
                # self.send_request_tf(ts)
                for i in range(leng + 1):
                    if name == "left":
                        count = leng - i if count_line % 2 == 1 else i
                    else:
                        count = i if count_line % 2 == 1 else leng - i
                    transform_stamped_msg = TransformStamped()
                    transform_stamped_msg.header.stamp = now
                    transform_stamped_msg.header.frame_id = f"tf{tf.child_frame_id}"
                    transform_stamped_msg.child_frame_id = str(count_plane) + '_' + str(
                        count_line) + "_" + str(count)
                    transform_stamped_msg.transform.rotation.x = 0.0
                    transform_stamped_msg.transform.rotation.y = 0.0
                    transform_stamped_msg.transform.rotation.z = 0.0
                    transform_stamped_msg.transform.rotation.w = 1.0
                    transform_stamped_msg.transform.translation.x = (-x_step * i + self.offset.x) * mode
                    transform_stamped_msg.transform.translation.y = .0 + self.offset.y
                    transform_stamped_msg.transform.translation.z = .0 + self.offset.z
                    self.tf_broadcaster.sendTransform(transform_stamped_msg)
                    self.tf_buffer.set_transform_static(transform_stamped_msg, str(count_plane) + '_' + str(
                        count_line) + "_" + str(count))
                    
        elif name == "up" or name == "bottom":
            # leng = int(np.around(step, 1) / y_step)
            # !!!!!!!!!!!!!!
            leng = 3
            print("step:", step)
            print("len", leng)
            for i, tf in enumerate(tf_last):
                # self.send_request_tf(ts)
                for count_line in range(leng):
                    if name == "bottom":
                        if leng % 2 == 1:
                            count = len(tf_last)-i-1 if count_line % 2 == 1 else i
                        else:
                            count = i if count_line % 2 == 1 else len(tf_last)-i-1
                    if name == "up":
                        count = len(tf_last)-i-1 if count_line % 2 == 1 else i
                            
                    # print(count_line, count, tf.child_frame_id)
                    transform_stamped_msg = TransformStamped()
                    transform_stamped_msg.header.stamp = now
                    transform_stamped_msg.header.frame_id = f"tf{tf.child_frame_id}"
                    # print(transform_stamped_msg.header.frame_id)
                    transform_stamped_msg.child_frame_id = str(count_plane) + '_' + str(count_line) + "_" + str(count)
                    transform_stamped_msg.transform.rotation.x = 0.0
                    transform_stamped_msg.transform.rotation.y = 0.0
                    transform_stamped_msg.transform.rotation.z = 0.0
                    transform_stamped_msg.transform.rotation.w = 1.0
                    transform_stamped_msg.transform.translation.x = .0 + self.offset.x
                    if name == "bottom":
                        transform_stamped_msg.transform.translation.y = -((count_line+1) * y_step) + self.offset.y
                    if name == "up":
                        transform_stamped_msg.transform.translation.y = (y_step*(leng)) - (count_line * y_step) + self.offset.y
                    transform_stamped_msg.transform.translation.z = .0 + self.offset.z
                    self.tf_broadcaster.sendTransform(transform_stamped_msg)
                    self.tf_buffer.set_transform_static(transform_stamped_msg, str(count_plane) + '_' + str(
                        count_line) + "_" + str(count))


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

    def distToPoint(self):
        distance = 99999.0
        name = None
        names = ["right", "bottom", "left", "up"]
        for item in names:
            tf = self.listener_tf_trajectory("manipulator", item)
            if tf is None:
                continue
            temp = math.sqrt(tf.transform.translation.x**2 + tf.transform.translation.y**2 + tf.transform.translation.z**2)
            print(item, ": ", temp)
            if temp < distance:
                distance = temp
                name = item
        return name, np.around(distance, 2)
    
    def get_last_point(self, array_slice, count_paint_make):
        name, distance = self.distToPoint()
        # print(name, ": ", distance)
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
        elif name == "up":
            if count_paint_make == 1:
                num = [f"{0}_{i}" for i in range(width)]
            else:
                num = [f"{count_paint_make-1}_{0}_{i}" for i in range(width)]
        elif name == "bottom":
            if count_paint_make == 1:
                num = [f"{height-1}_{i}" for i in range(width)]
            else:
                num = [f"{count_paint_make-1}_{height-1}_{i}" for i in range(width)]
        else:
            num = None
        #List names points tf
        print(num)
        if num != None:
            # Take last point from trajectory 
            last_points = [] 
            for item in num:
                last_points.append(self.listener_tf_trajectory('world', item))
            return last_points, name
        else:
            return None, None
    
    def generateTFPoints(self, last_point: list[TransformStamped], name: str, rotation: TransformStamped, array_slice: list, count_paint_make: int, y_step=0.3):
        width = len(array_slice[0])
        height = len(array_slice)
        now = self.get_clock().now().to_msg()
        last_points = []
        for i in range(height):
            ts = TransformStamped()
            ts.header.frame_id = "world"
            ts.header.stamp = now
            ts.child_frame_id = "tf" + str(count_paint_make) + str(i)
            ts.transform.translation = last_point[0].transform.translation
            if name == "right" or name == "left":
                ts.transform.translation.z -= y_step * i
            if name == "up":
                ts.transform.translation.z -= y_step * i - (y_step * (height-1))
            if name == "bottom":
                ts.transform.translation.z -= y_step * i + (y_step * (height-1))
            ts.transform.rotation = rotation
            last_points.append(ts)
            self.tf_buffer.set_transform_static(ts, "tf" + str(count_paint_make) + str(i))
            self.tf_broadcaster.sendTransform(ts)
        return last_points


    def send_goal(self, array_traject_msg: list):
        goal_msg = ExecuteTrajectoryArray.Goal()
        pi = math.pi
        for traject in array_traject_msg:
            goal_msg.trajectory_array.append(traject)
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

    def tf_eig_world(self, tf: TransformStamped):
        # tf_broadcaster = StaticTransformBroadcaster(self)
        now = self.get_clock().now()
        transform_stamped_msg2 = tf
        transform_stamped_msg2.header.stamp = now.to_msg()
        
        self.tf_broadcaster.sendTransform(transform_stamped_msg2)
        
        return transform_stamped_msg2
    
    def tf_listener_traject_manipulator(self) -> list[PoseArray]:
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
                # print("tf: ", tf_manipulator_point)
                count_point +=1 
                if tf_manipulator_point is False or tf_manipulator_point is None:
                    self.get_logger().info("Error manipulator TF!!!")
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
                    new_trajectory.poses.append(pose)
            array_traject_msg.append(new_trajectory)
            
        # tb = StaticTransformBroadcaster(self)
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
    
    def feedback_callback(self, feedback_msg):
        pass

    def status_callback(self, request: TrajectoryMode.Request, response: TrajectoryMode.Response):
        paint = test_driver_laser.PaintScanWall()
        print('______________________________________')
        mode_work = request.mode_work
        print(mode_work, type(mode_work))
        match mode_work:
            case ModeWork.G_CODE_MODE.value:
                # array_traject_msg = self.listener_tf_trajectory("manipulator", "world")
                path = request.path
                g = Gcode(path)
                trajectories = g.get_trajectories()
                array_traject_msg = []
                for array_lines in trajectories:
                    trajectory = TrajectoryMsg()
                    for point in array_lines:
                        if point.type == "MOVL":
                            command = CommandPose()
                            command.type = point.type
                            command.paint = point.paint
                            command.angle = .0
                            
                            start = PoseMsg()
                            start.x = point.pose.x / 1000
                            start.y = point.pose.y / 1000
                            start.z = point.pose.z / 1000
                            start.nx = math.radians(point.pose.nx)
                            start.ny = math.radians(point.pose.ny)
                            start.nz = math.radians(point.pose.nz)
                            
                            command.start = start
                            command.middle = PoseMsg()
                            command.end = PoseMsg()

                        if point.type == "MOVC":
                            command = CommandPose()
                            command.type = point.type
                            command.paint = point.paint
                            command.angle = point.angle
                            
                            start = PoseMsg()
                            start.x = point.start.x / 1000
                            start.y = point.start.y / 1000
                            start.z = point.start.z / 1000
                            start.nx = math.radians(point.start.nx)
                            start.ny = math.radians(point.start.ny)
                            start.nz = math.radians(point.start.nz)
                            
                            middle = PoseMsg()
                            middle.x = point.middle.x / 1000
                            middle.y = point.middle.y / 1000
                            middle.z = point.middle.z / 1000
                            middle.nx = math.radians(point.middle.nx)
                            middle.ny = math.radians(point.middle.ny)
                            middle.nz = math.radians(point.middle.nz)
                            
                            end = PoseMsg()
                            end.x = point.end.x / 1000
                            end.y = point.end.y / 1000
                            end.z = point.end.z / 1000
                            end.nx = math.radians(point.end.nx)
                            end.ny = math.radians(point.end.ny)
                            end.nz = math.radians(point.end.nz)
                            
                            command.start = start
                            command.middle = middle
                            command.end = end
                            pass
                        
                        trajectory.command.append(command)
                    array_traject_msg.append(trajectory)         
                print(array_traject_msg)   
                self.send_goal(array_traject_msg)
                response.success = True
                return response
            case ModeWork.SCAN_AND_PAINT.value:
                response_client = self.send_request(mode_work)
                # x_data, y_data, z_data = response_client.x_data, response_client.y_data, response_client.z_data
                # numpy_arr = paint.convert_srv_np(x_data, y_data, z_data)
                # print(numpy_arr)
                # pcd_new = paint.NumpyToPCD(numpy_arr)
                pcd_new = paint.PC2ToPCD(response_client.pc2[0])
                pcd_new = pcd_new.transform(np.array([[1, 0, 0, ParametrsManipulator.TRANS_PLANE.value - ParametrsManipulator.DIST_PAINT.value],
                                                        [0, 1, 0, 0],
                                                        [0, 0, 1, 0],
                                                        [0, 0, 0, 1]]))

                response.success = True
                return response
                        
            case ModeWork.CALCULATE_RANGE.value:
                self.send_request(mode_work)
                response.success = True
                response.message = "Calculate"
                return response

            case ModeWork.ONLY_PAINT.value:
                response.message += f"Дистанция до поверхности: {self.dist_point_to_plane}\n"
                if (self.dist_point_to_plane < ParametrsManipulator.MIN_DISTATION.value
                        or self.dist_point_to_plane > ParametrsManipulator.MAX_DISTATION.value):
                    response.message += "Дистанция слишком мала/велика"
                    response.message += "Передвиньте стрелу"
                if True:
                    if self.count_paint_make == 0:
                        if not self.mode:
                            global_tf_slice = self.tf_call_tool(array_slice=self.array_slice, euler=self.rotation)
                            self.send_request_angle(self.transform_eig_world, self.step)
                            self.mode = True
                        else:
                            match request.choose_start_manip:
                                case ChooseStartManip.START_MANIPULATOR.value:
                                    if not SelectModeWork.MANIPULATOR_SIMULATION.value:
                                        self.send_goal(self.array_traject_msg)
                                    self.count_paint_make +=1
                                    self.mode = False
                                case ChooseStartManip.SKIP_TRAJECTORY.value:
                                    self.mode = False
                                    pass   
                    else:
                        if not self.mode:
                            tf_last, name = self.get_last_point(self.array_slice, self.count_paint_make)
                            print("\n\n\n")
                            if name is not None:
                                self.tf_call_tool_rotate(tf_last, self.step, self.transform_eig_world.transform.rotation, self.count_paint_make, name)
                                self.array_traject_msg = self.tf_listener_traject_manipulator()
                                print('______________array_____________________')
                                print(self.array_traject_msg)
                                self.mode = True
                            else:
                                print("Error name reading!!!")
                        else:
                            match request.choose_start_manip:
                                case ChooseStartManip.START_MANIPULATOR.value:
                                    if not SelectModeWork.MANIPULATOR_SIMULATION.value:
                                        self.send_goal(self.array_traject_msg)
                                    self.count_paint_make +=1
                                    self.send_request_angle(self.transform_eig_world, self.step)
                                    self.mode = False
                                case ChooseStartManip.SKIP_TRAJECTORY.value:
                                    self.mode = False
                                    pass
                    response.success = True
                    return response
            case _:
                response.success = False
                response.message = "Error enter number!"
                return response

def main(args=None):
    rclpy.init(args=args)
    minimal_client = ServiceTrajectory()
    while KeyboardInterrupt:
        while rclpy.ok():
            rclpy.spin_once(minimal_client)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()