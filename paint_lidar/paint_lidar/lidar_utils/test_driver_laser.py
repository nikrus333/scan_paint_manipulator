import serial
import open3d as o3d
import time

from sympy import print_python
from .utils import hokuyo #
from .utils import serial_ports#
import math
import numpy as np
from numpy import linalg
from scipy.spatial.transform import Rotation as R
import copy
import random
from geometry_msgs.msg import Pose, PoseArray
from collections.abc import Sequence

from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PointStamped, Quaternion, Pose, PoseStamped
import tf2_geometry_msgs
#import tf_transformations as transformations

from .geometry_utils import GeometryMathMethods
from .enum_set import ParametrsManipulator, DevParametrs
import rclpy
from rclpy.node import Node






__author__ = 'niki'

def deg2rad(degrees):
    return degrees * (math.pi/180)

class HokuyoManipulator():
    def __init__(self) -> None:
        print('import test_driver_laser')
        uart_port = DevParametrs.LIDAR_DEV.value
        uart_speed = 19200
        self.laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
        self.port = serial_ports.SerialPort(self.laser_serial)
        self.laser = hokuyo.Hokuyo(self.port)
        self.laser.laser_on()
        self.o3d_pcd = o3d.geometry.PointCloud()
        
        self.trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

    def read_laser(self, T, R, T_tool_matrix  = [0.07, 0.0392, 0.335]):
        count_pcd = o3d.geometry.PointCloud()
        dict_laser = self.laser.get_single_scan()
        #print(dict_laser)
        points = []
        #print(T)
        #print(T[2] * 1000)
        T_tool_matrix =  [-0.07, 0.0392, 0.30]  #metrs x, y,z for manipulator

        for temp in dict_laser:
            #print(temp)
            z = math.cos(deg2rad(temp)) * dict_laser[temp] / 1000  + T_tool_matrix[2]   #metrs
            x = -math.sin(deg2rad(temp)) * dict_laser[temp] / 1000 + T_tool_matrix[0] 
            y = 0.0 + T_tool_matrix[1]
            points.append([x, y, z])
            #print(x, y, z)
            #print('----------------')
        #print(len(points))
        
        points = points[240:440]
        points = np.array(points)
        #self.o3d_pcd.points = o3d.utility.Vector3dVector(points)
        count_pcd.points = o3d.utility.Vector3dVector(points)
        copy_count_pcd = copy.deepcopy(count_pcd)
        R_tool_set = copy_count_pcd.get_rotation_matrix_from_xyz((0, 0, 0))
        copy_count_pcd.rotate(R_tool_set, center=(0, 0, 0))
        copy_count_pcd.rotate(R, center=(0, 0, 0))
        copy_count_pcd.translate(T)
    # o3d.visualization.draw_geometries([o3d_pcd])
        
        #pcd_final = count_pcd.transform(T)
        return copy_count_pcd

    def read_laser_scan_dist(self, T, R, T_tool_matrix  = [0.07, 0.0392, 0.335]):
        count_pcd = o3d.geometry.PointCloud()
        dict_laser = self.laser.get_single_scan()
        points = []
        T_tool_matrix =  [-0.07, 0.0392, 0.30]  #metrs x, y,z for manipulator
        
        for temp in dict_laser:
            #print(temp)
            z = math.cos(deg2rad(temp)) * dict_laser[temp] / 1000  + T_tool_matrix[2]   #metrs
            x = -math.sin(deg2rad(temp)) * dict_laser[temp] / 1000 + T_tool_matrix[0] 
            y = 0.0 + T_tool_matrix[1]
            points.append([x, y, z])
        points = points[330:370]
        filter_points = []
        for point in points:
            if point[2] >0.05 and point[2] < 5:
                filter_points.append(point)
        points = np.array(filter_points)
        
        count_pcd = o3d.geometry.PointCloud()
        count_pcd.points = o3d.utility.Vector3dVector(points)
        # o3d.visualization.draw_geometries([count_pcd])
        sum_y = 0
        for count, point in enumerate(points):
            sum_y += point[2]
        
        mean_y = sum_y / len(points) 

        return (mean_y +0.640) - (0.75 + 0.3 +0.55) 
    
    def coord_euler_to_matrix(self, trans : list, euler: list) -> np.ndarray:
        rotx = np.array([[1, 0, 0],
                    [0, math.cos(euler[2]), -math.sin(euler[2])],
                    [0, math.sin(euler[2]), math.cos(euler[2])]])

        roty = np.array([[math.cos(euler[1]), 0, math.sin(euler[1])],
                    [0, 1, 0],
                    [-math.sin(euler[1]), 0, math.cos(euler[1])]])

        rotz = np.array([[math.cos(euler[0]), -math.sin(euler[0]), 0],
                        [math.sin(euler[0]), math.cos(euler[0]), 0],
                        [0, 0, 1]])
        
        R = rotz@roty@rotx
    
        T_ = np.hstack((R, np.array([[trans[0]], [trans[1]], [trans[2]]])))
        

        T = np.vstack((T_, np.array([0,0,0,1])))

        T_new = np.array([[trans[0]], [trans[1]], [trans[2]]])
        #print(T)
        return T_new , R
    
    
   

     
class PaintScanWall():
    def __init__(self) -> None:
        pass
   

    def plane_segmentation(self, pcd):
        plane_model, inliers = pcd.segment_plane(distance_threshold=1,
                                         ransac_n=3,
                                         num_iterations=1000)
        [a, b, c, d] = plane_model
        #print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = pcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    def DetectMultiPlanes(self, points, min_ratio=0.05, threshold=0.01, iterations=1000) -> list:
        plane_list = []
        points = self.PCDToNumpy(points)
        N = len(points)
        target = points.copy()
        count = 0

        while count < (1 - min_ratio) * N:
            # w уравнение плоскости index нужные точки 
            w, index = self.PlaneRegression(
                target, threshold=threshold, init_n=3, iter=iterations)
        
            count += len(index)
            plane_list.append((w, target[index]))
            target = np.delete(target, index, axis=0)
        return plane_list

    def DrawPlanes(self, plane_list):
        results = plane_list
        planes = []
        colors = []
        count = 0
        if len(plane_list) < 4:
            for _, plane in results:
                match count:
                    case 0:
                        r = 1
                        g = 0
                        b = 0
                    case 1:
                        r = 0
                        g = 1
                        b = 0
                    case 2:
                        r = 0
                        g = 0
                        b = 1
                    case 3:
                        r = 1
                        g = 1
                        b = 0
                count +=1
                color = np.zeros((plane.shape[0], plane.shape[1]))
                color[:, 0] = r
                color[:, 1] = g
                color[:, 2] = b

                planes.append(plane)
                colors.append(color)


        else:
            for _, plane in results:

                r = random.random()
                g = random.random()
                b = random.random()

                color = np.zeros((plane.shape[0], plane.shape[1]))
                color[:, 0] = r
                color[:, 1] = g
                color[:, 2] = b

                planes.append(plane)
                colors.append(color)
    
        planes = np.concatenate(planes, axis=0)
        colors = np.concatenate(colors, axis=0)
        self.DrawResult(planes, colors)

    def PlaneRegression(self, points, threshold=0.01, init_n=3, iter=1000):
        pcd = self.NumpyToPCD(points)
        w, index = pcd.segment_plane(threshold, init_n, iter)

        return w, index
    
    def DrawResult(self, points, colors):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])
    
    def PCDToNumpy(self, pcd):
        return np.asarray(pcd.points)
    
    def NumpyToPCD(self, xyz):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd
       
    def CreateTraectory_circle_vertical(self, pcd_list) -> None:
        def find_eig_frame(pcd):        
            means, cov = pcd.compute_mean_and_covariance()
            w, v = linalg.eig(cov)
           # print(w)
            w_max = 0
            w_min = 0
            index_min = np.argmin(w)
            index_max = np.argmax(w)
            x_axis = np.array((1, 0, 0))
            z_direction = x_axis.dot(v[:, index_min])
            #print('z_direction', z_direction)
            if z_direction < 0:
                vecz = -v[:, index_min]
            else:
                vecz = v[:, index_min]
            vecx_ = v[:, index_max]
            vecy_ = np.cross(vecz, vecx_)
            z_axix_global = np.array((0, 0, 1))
            y_direction_1 = z_axix_global.dot(vecx_)
            y_direction_2 = z_axix_global.dot(vecy_)
            #print('y_direction', y_direction_1, y_direction_2)
            #print('index_min', index_min)
            if abs(y_direction_1) > abs(y_direction_2):
                if y_direction_1 > 0:
                    vecy = vecx_
                    vecx = vecy_
                else:
                    vecy = -vecx_
                    vecx = -vecy_

            else:
                if y_direction_2 > 0:
                    vecy = vecy_
                    vecx = vecx_
                else:
                    vecy = -vecy_
                    vecx = -vecx_
            #vec3 = np.cross(vec1, vec2)
            vec3 = v[:, index_min]
            Rot_mat = np.array([vecx, vecy, vecz]).T
            # Rot_mat = Rot_mat.dot(np.array([[math.cos(math.radians(180)), 0, math.sin(math.radians(180))],
            #                          [0, 1, 0],
            #                          [-math.sin(math.radians(180)), 0, math.cos(math.radians(180))]]))
            #Rot_mat = Rot_mat.dot(np.array([[1, 0, 0],
             #                       [0, math.cos(math.radians(-90)), -math.sin(math.radians(-90))],
              #                       [0, math.sin(math.radians(-90)), math.cos(math.radians(-90))]]))
            #print("det")
            #print(linalg.det(Rot_mat))

            return Rot_mat, means, v, w
        
        x_0, y_0, z_0 = 0.0, 0.0, 0.151
        R_sphere = ParametrsManipulator.SPHERE_RADIUS.value
        
        w, points = pcd_list[0]
        a, b, c, d = w
        pcd = self.NumpyToPCD(np.array(points))
        rotation, trans, v, w1 = find_eig_frame(pcd=pcd)
        
        dist_point_to_plane = abs(a * x_0 + b * y_0 + c * z_0 + d) / math.sqrt(a**2 + b**2 + c**2)
        dist_point_to_plane = dist_point_to_plane 
        T_pcd_ = np.hstack((rotation, np.array([[trans[0]], [trans[1]], [trans[2]]])))
        T_pcd = np.vstack((T_pcd_, np.array([0,0,0,1])))
        T_inv = linalg.inv(T_pcd)

        #print('dist to plane', dist_point_to_plane)
        x_B = x_0 + a*dist_point_to_plane
        y_B = y_0 + b*dist_point_to_plane
        z_B = z_0 + c*dist_point_to_plane
        #print(x_B, y_B, z_B)

        radius_circle = math.sqrt(R_sphere**2 - dist_point_to_plane**2)
        points = []
        points.append([x_B, y_B, z_B])
        points = np.array(points)
        pcd = self.NumpyToPCD(points)
    
        pcd = pcd.transform(T_inv)
        np_points_xy = self.PCDToNumpy(pcd)
        len_sqr = 2 * radius_circle / math.sqrt(2)
        min_x = np_points_xy[0][0] - len_sqr/2
        max_x = np_points_xy[0][0] + len_sqr/2
        min_y = np_points_xy[0][1] - len_sqr/2
        max_y = np_points_xy[0][1] + len_sqr/2
        #print(np_points_xy[0][0], np_points_xy[0][1], np_points_xy[0][2])
        y_step = 0.3
        x_step = 0.03
        min_y += 0.02
        min_x += 0.02
        x_dist = min_x
        x_dist_max = max_x
        y_dist = min_y
        slise_traject = []
        array_slice_traject = []
        flag_left_side = True
        flag_traject = 0
        while max_y > min_y:
            flag_traject += 1
            if flag_traject > 4:
                break
            if flag_left_side:
                while min_x < max_x: 
                    slise_traject.append([(min_x), (max_y), (0.0)])
                    min_x += x_step
                min_x = x_dist
                flag_left_side = False
            else:
                while max_x > min_x: 
                    slise_traject.append([(max_x), (max_y), (0.0)])
                    max_x -= x_step
                max_x = x_dist_max
                flag_left_side = True
            #print(slise_traject)
            slise_traject_pcd = self.NumpyToPCD(np.array(slise_traject))
            slise_traject_pcd_rotate = slise_traject_pcd.transform(T_pcd)
            slise_traject = self.PCDToNumpy(slise_traject_pcd_rotate)
            array_slice_traject.append(slise_traject)
            #print('array_slice_traject ', array_slice_traject)
            slise_traject = []
            
            max_y -= y_step
        #print('point_ceel ',point_ceel)
        points0 , points1 = array_slice_traject[0][0] , array_slice_traject[0][2]
        #print('normale' , a, b, c)
        euler = self.orient_to_euler(np.array([a, b, c]), np.array(points0), np.array(points1))
        step = len_sqr 

        return array_slice_traject, euler, rotation, trans, dist_point_to_plane, step
    
   
    def orient_to_euler(self, plane_norm, pt1, pt2):
        tangent = pt2 - pt1
        norm_tangent = tangent / linalg.norm(tangent)
        R_mat = np.array([np.cross(plane_norm, norm_tangent), norm_tangent, plane_norm]).T

        # R_mat матрица поворачивает ориентацию в точках окраски

        # R_mat = R_mat.dot(np.array([[1, 0, 0],
        #                             [0, math.cos(math.radians(-90)), -math.sin(math.radians(-90))],
        #                             [0, math.sin(math.radians(-90)), math.cos(math.radians(-90))]]))

        # R_mat = R_mat.dot(np.array([[math.cos(math.radians(-90)), -math.sin(math.radians(-90)), 0],
        #                            [math.sin(math.radians(-90)), math.cos(math.radians(-90)), 0],
        #                             [0, 0, 1]]))
        R_mat = R_mat.dot(np.array([[1, 0, 0],
                                     [0, math.cos(math.radians(-180)), -math.sin(math.radians(-180))],
                                     [0, math.sin(math.radians(-180)), math.cos(math.radians(-180))]]))
        # вращение по y
        # R_mat = R_mat.dot(np.array([[math.cos(math.radians(180)), 0, math.sin(math.radians(180))],
        #                            [0, 1, 0],
        #                            [-math.sin(math.radians(180)), 0, math.cos(math.radians(180))]]))
        euler_angle = self.rot2euler(R_mat)
        return euler_angle
    
    def rot2euler(self, rot: np.ndarray) -> np.ndarray:
        sy = math.sqrt(rot[0,0] * rot[0,0] +  rot[1,0] * rot[1,0])
    
        singular = sy < 1e-6
        if  not singular :
            x = math.atan2(rot[2,1] , rot[2,2])
            y = math.atan2(-rot[2,0], sy)
            z = math.atan2(rot[1,0], rot[0,0])
        else :
            x = math.atan2(-rot[1,2], rot[1,1])
            y = math.atan2(-rot[2,0], sy)
            z = 0
    
        return np.array([z, y, x])
    
    

    def convert_np_srv(self, pcd_arr):
        #print(pcd_arr)
        x_arr, y_arr, z_arr = [], [], []
        for point in pcd_arr:
            x_arr.append(point[0])
            y_arr.append(point[1])
            z_arr.append(point[2])
        return x_arr, y_arr, z_arr

    def convert_srv_np(self, x_data, y_data, z_data):
        points = []
        for count in range(len(x_data)):
            point = [x_data[count], y_data[count], z_data[count]]
            points.append(point)
        numpy_arr = np.array(points)
        return numpy_arr

    def select_plane(self, plane_list):
        print('Введите номер поверхности, которую хотите красить')
        print('1 - red')
        print('2 - green')
        print('3 - blue')
        nomber_plane =  int(input())
        plane_list = [plane_list.pop(nomber_plane-1)]
        return plane_list
    
    @classmethod
    def sum_tf_stemp(cls, tf1, tf2, time):
        
        
        tf_new = TransformStamped()
        tf_new.header.stamp = time
        tf_new.header.frame_id = 'world'
        tf_new.child_frame_id = 'eig1'
        tf_new.transform.translation.x = tf1.transform.translation.x + tf2.transform.translation.x
        tf_new.transform.translation.y = tf1.transform.translation.y + tf2.transform.translation.y 
        tf_new.transform.translation.z = tf1.transform.translation.z + tf2.transform.translation.z
        #tf_new.transform.rotation = tf1.transform.rotation * tf2.transform.rotation

        msg_quat_1 = Quaternion(x=tf1.transform.rotation.x, y=tf1.transform.rotation.y, 
                                z=tf1.transform.rotation.z, w=tf1.transform.rotation.w)
        msg_quat_2 = Quaternion(x=tf2.transform.rotation.x, y=tf2.transform.rotation.y, 
                                z=tf2.transform.rotation.z, w=tf2.transform.rotation.w)
        mgq_quat_new = cls.multiplay_quaternion(tf1.transform.rotation, tf2.transform.rotation)
        tf_new.transform.rotation = mgq_quat_new

        # new way
        q_1 = [tf1.transform.rotation.x, tf1.transform.rotation.y, tf1.transform.rotation.z, tf1.transform.rotation.w]
        transform_matrix_1 = GeometryMathMethods.transformation_matrix(tf1.transform.translation.x, 
                                                                       tf1.transform.translation.y, tf1.transform.translation.z,
                                                                       q_1)
        print(tf1.transform.translation.y)
        
        q_2 = [tf2.transform.rotation.x, tf2.transform.rotation.y, tf2.transform.rotation.z, tf2.transform.rotation.w]
        transform_matrix_2 = GeometryMathMethods.transformation_matrix(tf2.transform.translation.x, 
                                                                       tf2.transform.translation.y, tf2.transform.translation.z,
                                                                       q_2)
        
        end_matrix = np.dot(transform_matrix_1, transform_matrix_2)
        x, y, z ,q = GeometryMathMethods.retrieve_coordinates_and_quaternion(end_matrix)
        tf_new.transform.translation.x = x
        tf_new.transform.translation.y = y
        tf_new.transform.translation.z = z
        #mgq_quat_new = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        #tf_new.transform.rotation = mgq_quat_new
        return tf_new
    
    @classmethod
    def multiplay_quaternion(cls, a, b):
            w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z  #1
            x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y  #i
            y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x  #j
            z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w   #k
            return Quaternion(x=x, y=y, z=z, w=w)
    

    
if __name__ == '__main__':
    lidar = HokuyoManipulator()
    while True:
        print(lidar.read_laser_scan_dist(1 ,1))
        time.sleep(1)
