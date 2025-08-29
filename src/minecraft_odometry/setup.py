from setuptools import setup
import os
from glob import glob

package_name = 'minecraft_odometry'

setup(
    name='minecraft_odometry',
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*launch*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Odometry converter for Minecraft ROS2 integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_converter = minecraft_odometry.odometry_converter:main',
            'sensor_synchronizer = minecraft_odometry.sensor_synchronizer:main',
            'sensor_synchronizer_v2 = minecraft_odometry.sensor_synchronizer_v2:main',
            'simple_synchronizer = minecraft_odometry.simple_synchronizer:main',
        ],
    },
)