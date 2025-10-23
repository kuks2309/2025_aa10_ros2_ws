from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'handsfree_ros_imu_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HandsFree',
    maintainer_email='hands_free@126.com',
    description='HandsFree IMU ROS2 driver package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hfi_a9_ros_ros2 = handsfree_ros_imu_ros2.hfi_a9_ros_ros2:main',
        ],
    },
)