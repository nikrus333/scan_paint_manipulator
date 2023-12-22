#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point32, TransformStamped
import open3d as o3d
import numpy as np
import math
from std_msgs.msg import Header
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Node has been started.")
        self.main()

    def main(self):
        self.pcd = o3d.geometry.PointCloud()
        self.pcd = o3d.io.read_point_cloud("/home/alex/ros2_scan/src/scan_paint_manipulator/paint_lidar/scan_obj/1.pcd")
        pointcloud2 = self.PCDToPC2(self.pcd, "base_link")
        array = self.PC2ToNumpy(pointcloud2)
        print("PC2 -> Numpy:", array)
        print("PCD -> Numpy:", self.PCDToNumpy(self.pcd))
        # self.pub = self.create_publisher(PointCloud2, "/point_cloud", qos_profile=10)
        # self.pub.publish(pointcloud2)
        # self.DrawResult(self.pcd)

    def createTFPoints(self, list_tf:list[TransformStamped], count_plane: int, rotation):
        tf_broadcaster = TransformBroadcaster()
        for tf in list_tf:
            tf_name = f"tf{tf.child_frame_id}"
            ts = TransformStamped()
            ts.header.frame_id = "map"
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.child_frame_id = tf_name
            ts.transform.translation = tf.transform.translation
            ts.transform.rotation = rotation
            tf_broadcaster.sendTransform(ts)

    def generate_circle_points(self, radius: float, height: int, num_points: int):
        points = []
        angle_increment = (2 * 3.14159) / num_points
        for j in range(int(height)*10):
            for i in range(num_points):
                angle = i * angle_increment
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                z = j / 10
                points.append([x, y, z])
        return points

    def PCDToPC2(self, pcd, frame_id: str) -> PointCloud2:
        array = self.PCDToNumpy(pcd)
        return self.NumpyToPC2(array, frame_id)

    def NumpyToPC2(self, array_list: list[list[int]], frame_id: str) -> PointCloud2:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = header
        point_cloud_msg.height = 1
        point_cloud_msg.width = len(array_list)
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.is_dense = True

        point_field = PointField()
        point_field.name = "x"
        point_field.offset = 0
        point_field.datatype = PointField.FLOAT32
        point_field.count = 1
        point_cloud_msg.fields.append(point_field)

        point_field = PointField()
        point_field.name = "y"
        point_field.offset = 4
        point_field.datatype = PointField.FLOAT32
        point_field.count = 1
        point_cloud_msg.fields.append(point_field)

        point_field = PointField()
        point_field.name = "z"
        point_field.offset = 8
        point_field.datatype = PointField.FLOAT32
        point_field.count = 1
        point_cloud_msg.fields.append(point_field)

        point_cloud_msg.point_step = 12
        point_cloud_msg.row_step = len(array_list) * point_cloud_msg.point_step
        point_cloud_msg.data = []

        for point in array_list:
            point_cloud_msg.data.extend(struct.pack('fff', point[0], point[1], point[2]))

        return point_cloud_msg
    
    def PC2ToNumpy(self, pointcloud2: PointCloud2) -> np.ndarray:
        point_step = pointcloud2.point_step
        data = pointcloud2.data
        point_array = []
        for i in range(0, len(data), point_step):
            x = np.around(struct.unpack_from('f', data, i)[0], 8)
            y = np.around(struct.unpack_from('f', data, i + 4)[0], 8)
            z = np.around(struct.unpack_from('f', data, i + 8)[0], 8)
            point_array.append([x, y, z])
        return point_array

    def DrawResult(self, pcd):
        o3d.visualization.draw_geometries([pcd])
    
    def PCDToNumpy(self, pcd):
        return np.asarray(pcd.points)

    def NumpyToPCD(self, xyz: np.ndarray) -> o3d.geometry.PointCloud:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd

    def PCDToNumpy(self, pcd):
        return np.asarray(pcd.points)
    # ----------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()