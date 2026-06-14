import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'r2_odin_pose_pid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xie',
    maintainer_email='a15992282905@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odin_pose_pid_mock_server = r2_odin_pose_pid.odin_pose_pid_mock_server:main',
            'odin_pose_pid_server = r2_odin_pose_pid.odin_pose_pid_server:main',
        ],
    },
)
