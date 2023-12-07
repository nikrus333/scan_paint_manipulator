import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Point
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException


from example_interfaces.srv import PoseTf

class TfCreatorDataBase():
    def __init__(self, clock) -> None:
        self.array_tf = []
        self._clock = clock

    def add_tf(self, pose : Pose, child_name, parent_name) -> None:
        t = TransformStamped()
        t.header.stamp = self._clock
        t.header.frame_id = str(parent_name)
        t.child_frame_id = str(child_name)
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w
        self.array_tf.append(t)
        

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('tf_database_node')
        self.srv = self.create_service(PoseTf, '/service_send_tf', self.status_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_creator_db = TfCreatorDataBase(self.get_clock().now().to_msg())

    def listener_tf_trajectory(self, parent: str, child: str) -> Pose:
        try:
            t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation.x = t.transform.rotation.x
            pose.orientation.y = t.transform.rotation.y
            pose.orientation.z = t.transform.rotation.z
            pose.orientation.w = t.transform.rotation.w
            self.get_logger().info("yes listen")
            return pose
        except TransformException as ex:    
            self.get_logger().info(f'Could not transform {parent} to {child}: {ex}') 
            return None

    def timer_callback(self):
        self.get_logger().info(f'No dada') 
        for count, tf in enumerate(self.tf_creator_db.array_tf):
            tf.header.stamp = self.get_clock().now().to_msg()        
            self.tf_broadcaster.sendTransform(tf)
            self.get_logger().info(f"yes {len(self.tf_creator_db.array_tf)}")

    def status_callback(self, request: PoseTf.Request, response: PoseTf.Response):
        pose = request.pose
        child_name = request.child_name
        parent_name = request.parent_name
        self.tf_creator_db.add_tf(pose, child_name, parent_name)
        self.get_logger().info(f"Request yes")
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    pose = Pose()
    #minimal_publisher.tf_creator_db.add_tf(pose, '0_15', 'manipulator')
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()