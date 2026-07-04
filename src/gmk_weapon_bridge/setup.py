import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gmk_weapon_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), ['setup_network.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TECHX Robocon',
    maintainer_email='techx@example.local',
    description='ROS2 UDP bridge for TECHX weapon quick-coupler vision targets.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = gmk_weapon_bridge.bridge_node:main',
            'mock_jetson_sender = gmk_weapon_bridge.mock_jetson_sender:main',
            'verify_udp = gmk_weapon_bridge.verify_udp:main',
        ],
    },
)
