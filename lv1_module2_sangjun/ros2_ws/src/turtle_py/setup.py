import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtle_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pa31',
    maintainer_email='hsj38038@gmail.com',
    description='ROS 2 package for turtlesim control and custom interfaces',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'distance_node = turtle_py.distance_node:main',
            'warning_node = turtle_py.warning_node:main',
            'square_node = turtle_py.square_node:main',
            'turtle_distance_publisher = turtle_py.ex03_distance_publisher:main',
            'turtle_distance_subscriber = turtle_py.ex03_distance_subscriber:main',
            'builtin_service_client = turtle_py.ex05_builtin_service_client:main',
            'toggle_servers = turtle_py.ex05_toggle_servers:main',
            'rotate_absolute_client = turtle_py.ex05_rotate_absolute_client:main',
            'polygon_action_server = turtle_py.ex06_polygon_action_server:main',
            'waypoint_publisher = turtle_py.ex06_waypoint_publisher:main',
            'qos_sensor_publisher = turtle_py.ex07_qos_sensor_publisher:main',
            'qos_subscriber = turtle_py.ex07_qos_subscriber:main',
        ],
    },
)

