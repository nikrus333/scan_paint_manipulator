#!/usr/bin/env python

# from manipulator_control.srv import MoveSrv                            # CHANGE
import sys
import rclpy
from rclpy.node import Node
from math import pi, sqrt
from example_interfaces.action import ExecuteTrajectoryArray
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, PoseArray

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # self.cli = self.create_client(MoveSrv, '/MoveL')       # CHANGE
        self.action_client = ActionClient(self, ExecuteTrajectoryArray, 'execute_laser_trajectory')
        self.pub_state = self.create_publisher(Pose, "robot_state", 5)

    def send_goal(self):

        goal_msg = ExecuteTrajectoryArray.Goal()

        pose_arr1 = PoseArray()
        pose_arr2 = PoseArray()

        pose1 = Pose()
        pose2 = Pose()
        # pose1.position.x = 0.25
        # pose1.position.y = -0.7
        # pose1.position.z = 0.07

        pose1.position.x = -0.02440611
        pose1.position.y = -0.59178486
        pose1.position.z = 0.07
        r1 = R.from_euler('zyx', [pi/2, 0, pi])
        x,y,z,w = r1.as_quat()

        pose1.orientation.x = x
        pose1.orientation.y = y
        pose1.orientation.z = z
        pose1.orientation.w = w
        

        pose2.position.x = -0.02440611
        pose2.position.y = -0.7
        pose2.position.z = 0.07



        pose2.orientation.x = x
        pose2.orientation.y = y
        pose2.orientation.z = z
        pose2.orientation.w = w
        pose_arr1.poses = [pose1, pose2]
        pose_arr2.poses = [pose1, pose2]
        
        goal_msg.poses_array = [pose_arr1, pose_arr2]

        goal_msg.acceleration = 0.1
        goal_msg.velocity = 0.1

        self.action_client.wait_for_server()

        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        #return self.action_client.send_goal_async(goal_msg)
        # self.req.cmd1 = [0.25, -0.25]
        # self.req.cmd2 = [-0.7, -0.7]
        # self.req.cmd3 = [0.07, 0.07]
        # self.req.cmd4 = [179.999 * pi / 180, 179.999 * pi / 180]
        # self.req.cmd5 = [0.0027 * pi / 180, 0.0027 * pi / 180]
        # self.req.cmd6 = [179.999 * pi / 180, 179.999 * pi / 180]
        # self.req.velocity = float(0.1)
        # self.req.acceleration = float(
        #     0.1)  # ANGE



        # self.req.cmd1 = [0.16315028781231988, 0.1611153637133262]
        # self.req.cmd2 = [-0.65, -0.3]
        # self.req.cmd3 = [0.16315028781231988,   0.1611153637133262]

        # self.req.cmd4 = [179.999 * pi / 180, 179.999 * pi / 180]
        # self.req.cmd5 = [0.0027 * pi / 180, 0.0027 * pi / 180]
        # self.req.cmd6 = [179.999 * pi / 180, 179.999 * pi / 180]
        # self.req.velocity = float(0.1)
        # self.req.acceleration = float(
        #     0.1)  # ANGE


        # self.req.cmd1 = [0.25, 0.127]
        # self.req.cmd2 = [-0.41, 0.465]
        # self.req.cmd3 = [0.07, 0.1273]
        # self.req.cmd4 = [0.0, 0.0]
        # self.req.cmd5 = [0.0, 1.0]
        # self.req.cmd6 = [1.0, 0.0]
        # self.req.velocity = float(0.1)
        # self.req.acceleration = float(
        #     0.1)  # ANGE


        # self.req.cmd1 = [-0.446]
        # self.req.cmd2 = [-0.72]
        # self.req.cmd3 = [0.043905484342596335]
        # self.req.cmd4 = [pi]
        # self.req.cmd5 = [0.0]
        # self.req.cmd6 = [pi]
        # self.future = self.cli.call_async(self.req)
        # print("call")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        state = Pose()
        state.position.x = feedback.robot_state[0]
        state.position.y = feedback.robot_state[1]
        state.position.z = feedback.robot_state[2]
        
        r = R.from_euler('zyx', [feedback.robot_state[3], feedback.robot_state[4], feedback.robot_state[5]])

        x,y,z,w = r.as_quat()

        state.orientation.x = x
        state.orientation.y = y
        state.orientation.z = z
        state.orientation.w = w

        self.pub_state.publish(state)

def main(args=None):
    rclpy.init(args=args)
    action_client = MinimalClientAsync()

    action_client.send_goal()
    rclpy.spin(action_client)

    #rclpy.shutdown()


if __name__ == '__main__':
    main()
