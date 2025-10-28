from setuptools import setup
import os
from glob import glob

package_name = 'ai_line_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amap',
    maintainer_email='amap@todo.todo',
    description='AI-based line detection using YOLOv8 segmentation for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_line_detection_node = ai_line_detection.ai_line_detection_node:main',
            'image_publisher_node = ai_line_detection.image_publisher_node:main',
        ],
    },
)
