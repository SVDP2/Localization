from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'aruco_imu_eskf_localization'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'filterpy'],
    zip_safe=False,
    maintainer='xytron',
    maintainer_email='xytron@example.com',
    description='ArUco board detection and IMU-based relative localization.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_detector_node = '
            'aruco_imu_eskf_localization.aruco_detector_node:main',
            'relative_localization_node = '
            'aruco_imu_eskf_localization.relative_localization_node:main',
            'tf_distance_viz_node = '
            'aruco_imu_eskf_localization.tf_distance_viz_node:main',
        ],
    },
)
