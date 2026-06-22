from glob import glob
import os

from setuptools import setup

package_name = "r2_vision_servo"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xie",
    maintainer_email="a15992282905@gmail.com",
    description="R2 upper-computer visual servo action server.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_servo_action_server = r2_vision_servo.vision_servo_action_server:main",
        ],
    },
)
