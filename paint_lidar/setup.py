import os
from glob import glob
from setuptools import setup

package_name = 'paint_lidar'
submodules = "paint_lidar/lidar_utils"
submodules_1 = "paint_lidar/lidar_utils/utils"
submodules_2 = "paint_lidar/manip_utils"
#submodules_3 = "paint_lidar/manip_utils"
#submodules_3 = "paint_lidar/libs"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules, submodules_1, submodules_2],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nik',
    maintainer_email='nikrus333@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'paint_wall = paint_lidar.paint_wall:main',
            'cliente = paint_lidar.servece_send:main',
            'nozzle_close_open = paint_lidar.nozzle_node:main',
            'service_nozzle = paint_lidar.service_nozzle:main',
            'client_opening = paint_lidar.client_opening:main',
            'tf_pub = paint_lidar.tf_srv_pub:main',
            'fibonacci_action_service = paint_lidar.action_service:main',
            'service_trajectory = paint_lidar.service_trajectory:main',
            'menu = paint_lidar.menu:main',
            'example_pcl2 = paint_lidar.example_pcl2:main',
        ],
    },
)
