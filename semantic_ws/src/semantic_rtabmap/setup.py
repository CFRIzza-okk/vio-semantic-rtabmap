from glob import glob
import os

from setuptools import find_packages, setup

package_name = "semantic_rtabmap"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jetson",
    maintainer_email="jetson@localhost",
    description="为 RTAB-Map 增加语义能力的外挂语义层",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = semantic_rtabmap.detector_node:main",
            "mapper_node = semantic_rtabmap.mapper_node:main",
            "cmd_vel_safety_node = semantic_rtabmap.cmd_vel_safety_node:main",
            "ros2_relay_node = semantic_rtabmap.ros2_relay_node:main",
            "obstacle_monitor_node = semantic_rtabmap.safety_monitor_node:main",
            "localization_monitor_node = semantic_rtabmap.safety_monitor_node:main_localization",
            "exploration_node = semantic_rtabmap.exploration_node:main",
            "point_cloud_inspector_node = semantic_rtabmap.point_cloud_inspector_node:main",
        ],
    },
)
