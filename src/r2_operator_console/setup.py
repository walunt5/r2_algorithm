from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'r2_operator_console'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xie',
    maintainer_email='a15992282905@gmail.com',
    description='R2 operator console for manual driving and action clients.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'operator_console = r2_operator_console.operator_console:main',
        ],
    },
)
