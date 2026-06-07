import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'techx_r2_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
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
            'protocol_demo = techx_r2_arm_control.protocol_demo:main',
            'arm_serial_node = techx_r2_arm_control.arm_serial_node:main',
            'test_get_head_client = techx_r2_arm_control.test_get_head_client:main',
            'test_set_end_effector_client = techx_r2_arm_control.test_set_end_effector_client:main',
            'test_execute_action_client = techx_r2_arm_control.test_execute_action_client:main',
            'mock_arm_stm32 = techx_r2_arm_control.mock_arm_stm32:main',
        ],
    },
)
