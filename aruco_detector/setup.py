from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='user1',
    maintainer_email='kikiws70@gmail.com',
    description='ArUco marker detector for ROS2',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'aruco_detector_node = aruco_detector.aruco_detector_node:main',
            'tf_distance_viz_node = aruco_detector.tf_distance_viz_node:main',
        ],
    },
)
