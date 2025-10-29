from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_toolbox_config'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amap',
    maintainer_email='kuks2309@hotmail.com',
    description='SLAM Toolbox configuration package for AA10 Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_reset_service = slam_toolbox_config.simple_reset_service:main',
        ],
    },
)
