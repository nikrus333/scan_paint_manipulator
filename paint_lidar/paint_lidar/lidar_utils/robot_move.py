import motorcortex
import math
import time
from robot_control import to_radians
from robot_control.motion_program import Waypoint, MotionProgram, PoseTransformer
from robot_control.robot_command import RobotCommand
from robot_control.system_defs import InterpreterStates
import numpy as np
import cv2
import open3d as o3d
import numpy as np
import time
import matplotlib.cm as plt
import random
import copy
import test_driver_laser


class PaintScanWall():
    def __init__(self) -> None:
        pass
    def calculate_traectories(self, pcd : o3d.geometry.PointCloud):
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array( pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        #o3d.visualization.draw_geometries([pcd])

    def plane_segmentation(self, pcd):
        plane_model, inliers = pcd.segment_plane(distance_threshold=1,
                                         ransac_n=3,
                                         num_iterations=1000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = pcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    def DetectMultiPlanes(self, points, min_ratio=0.05, threshold=0.01, iterations=1000):


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
    
    def CreateTraectory(self, pcd_list) -> None:
        min_value = []
        max_value = []
        traectory_cell = []
        #find
        
        for pcd in pcd_list:
            min_x, min_y, min_z = 10000
            max_x, max_y, max_z = -10000
            
            w, points = pcd
            a, b, c, d = w
            angle_plate = math.cos((a + b)/((math.sqrt(a*a + b*b + c*c)) * (math.sqrt(2))))
            R = pcd.get_rotation_matrix_from_xyz((np.pi/2-angle_plate, 0, 0))
            outlier_cloud = pcd.rotate(R, center=(0,0,0))
            for point in outlier_cloud:
                if point[0] < min_x:
                    min_x = point[0]
                if point[1] < min_y:
                    min_y = point[1]
                if point[2] < min_z:
                    min_z = point[2]
                
                if point[0] > max_x:
                    max_x = point[0]
                if point[1] > max_y:
                    max_y = point[1]
                if point[2] > max_z:
                    max_z = point[2]

            min_value.append([min_x, min_y, min_z])
            max_value.append([max_x, max_y, max_z])

            point_ceel = []
            z_mean = (max_z + min_z) / 2
            for y_count in range(max_y - min_y):
                for x_count in range(max_x - min_x):
                    point_ceel.append([x_count + min_x, y_count + min_y, z_mean])

            R = pcd.get_rotation_matrix_from_xyz((np.pi/2-angle_plate, 0, 0))
            pcd_cell = pcd.rotate(R, center=(0,0,0))
            traectory_cell.append(outlier_cloud)
        
        return traectory_cell

    def send_data(self, pcd_list):
        pass



def main():
    hok = test_driver_laser.HokuyoManipulator()

    # create visualizer and window.
    parameter_tree = motorcortex.ParameterTree()
    motorcortex_types = motorcortex.MessageTypes()
    motorcortex_msg = motorcortex_types.motorcortex()

    req, sub = motorcortex.connect('wss://192.168.5.85:5568:5567', 
                                    motorcortex_types, 
                                    parameter_tree,timeout_ms=1000, 
                                    certificate="motorcortex-robot-control-python/test/mcx.cert.pem",
                                    login="admin",password="vectioneer")

    subscription = sub.subscribe(['root/Control/fkToolSetPoint/toolCoordinates'], 'group1', 5)
    subscription.get()

    # initialize pointcloud instance.
    pcd = o3d.geometry.PointCloud()
    # *optionally* add initial points
    points = np.random.rand(10, 3)
    points = []


    points.append([0.0, 0.0, 1.0])
    points = np.array(points)
    pcd.points = o3d.utility.Vector3dVector(points)

    # include it in the visualizer before non-blocking visualization.


    # to add new points each dt secs.
    dt = 0.01
    # number of points that will be added
    n_new = 10

    previous_t = time.time()

    # run non-blocking visualization. 
    # To exit, press 'q' or click the 'x' of the window.
    keep_running = False
    try:
        while keep_running:
            if time.time() - previous_t > dt:
                # Options (uncomment each to try them out):
                # 1) extend with ndarrays.
                params = subscription.read()
                #print(params)
                value = params[0].value
                if value == [0, 0, 0, 0, 0, 0]:
                    continue
                print(value)
                trans, euler = value[:3], value[3:]
                #print(trans)
                trans_init, R = hok.coord_euler_to_matrix(trans, euler)
                #print(hok.trans_init)
                #print(trans_init)
                time.sleep(1)
                pcd = pcd + hok.read_laser(trans_init, R)


                previous_t = time.time()
                #print(pcd.points)
                #o3d.visualization.draw_geometries([pcd])

    except KeyboardInterrupt:
        print('End process scan')
        req.close()
        sub.close()
    finally:
        #o3d.visualization.draw_geometries([pcd])     
        #o3d.io.write_point_cloud('1.pcd', pcd) # save pcd data

        pcd_new = o3d.io.read_point_cloud("1.pcd")
        #o3d.visualization.draw_geometries([pcd_new , o3d.geometry.TriangleMesh.create_coordinate_frame()])
        #o3d.visualization.draw_geometries([pcd_new])  
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame())
        vis.add_geometry(pcd_new)
        
        vis.run()
        vis.destroy_window()
        paint = PaintScanWall()
        #paint.calculate_traectories(pcd_new)
        #paint.plane_segmentation(pcd_new)
        plane_list = paint.DetectMultiPlanes(pcd_new, min_ratio=0.09, threshold=15, iterations=1000)
        print(len(plane_list))
        #first = plane_list.pop()
        #print(first)
        paint.DrawPlanes(plane_list)

        

if __name__ == "__main__":
    main()