from glob import glob
import os

from setuptools import setup


package_name = 'camera_lidar_extrinsic_calibrator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='xytron@example.com',
    description='Interactive follower camera-LiDAR extrinsic calibration tool.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_lidar_extrinsic_calibrator_node = '
            'camera_lidar_extrinsic_calibrator.camera_lidar_extrinsic_calibrator_node:main',
            'camera_lidar_manual_adjust_node = '
            'camera_lidar_extrinsic_calibrator.camera_lidar_manual_adjust_node:main',
        ],
    },
)
