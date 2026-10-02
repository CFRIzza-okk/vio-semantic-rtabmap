#!/usr/bin/env python3
"""semantic.launch.py — 语义感知层启动文件（Phase 1）

只负责语义层：静态 TF + semantic_detector 节点。
相机由外部启动（见 ~/VIO/launch_semantic_demo.sh）。

为什么这里不 include realsense 的 rs_launch.py
--------------------------------------------
realsense 的 rs_launch.py 里有这么一段：

    for param_name in context.launch_configurations.keys():
        params[param_name] = LaunchConfiguration(param_name)

它会把 **launch 上下文里所有已声明的参数** 都当作相机节点的参数传下去。
如果我们在同一个 launch 里声明 target_frame / model_path 之类的参数，
相机节点就会刷一屏 "Parameter 'xxx' is not supported" 警告。
把相机拆到独立的 launch 调用里，两边各有独立的 launch 上下文，问题消失。

用法示例
--------
离线自测（无 PX4，用静态 map->base_link 顶替 MAVROS 里程计）:
    ros2 launch semantic_rtabmap semantic.launch.py publish_static_map_tf:=true

接 PX4 实机（map 由 MAVROS 独占发布，绝不能加静态 TF）:
    ros2 launch semantic_rtabmap semantic.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("semantic_rtabmap")
    default_params = os.path.join(pkg_share, "config", "semantic.yaml")
    default_mapper_params = os.path.join(pkg_share, "config", "mapper.yaml")

    # ---------------- 启动参数 ----------------
    args = [
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="semantic_detector 的参数文件",
        ),
        DeclareLaunchArgument(
            "mapper_params_file", default_value=default_mapper_params,
            description="semantic_mapper 的参数文件",
        ),
        DeclareLaunchArgument(
            "publish_static_map_tf", default_value="false",
            description="无 PX4 时发布静态 map->base_link（仅供离线测试）",
        ),
        DeclareLaunchArgument(
            "publish_static_camera_tf", default_value="true",
            description=(
                "是否发布 base_link->camera_link 静态外参。"
                "使用手持建图脚本时设为 false —— 由脚本统一发布，避免两个发布者冲突"
            ),
        ),
        DeclareLaunchArgument(
            "persistence_path", default_value="",
            description="语义地图 JSON 保存路径；留空 = 不持久化",
        ),
        DeclareLaunchArgument(
            "target_frame", default_value="map",
            description="语义物体的目标坐标系",
        ),
        DeclareLaunchArgument(
            "model_path", default_value="/home/jetson/ultralytics/ultralytics/yolo11n.engine",
            description="YOLO 模型路径（.engine 优先，加载失败自动回退 .pt）",
        ),
        DeclareLaunchArgument(
            "class_filter", default_value="",
            description="类别白名单，逗号分隔；留空 = 全部 COCO 类别",
        ),
        DeclareLaunchArgument(
            "publish_debug_image", default_value="true",
            description="是否发布带检测框的调试图像",
        ),
        DeclareLaunchArgument(
            "process_rate", default_value="0.0",
            description=(
                "推理限速（Hz），0 = 不限速。语义建图不需要跑满相机帧率——"
                "物体是静止的，而 RTAB-Map 的回环检测也只有 2 Hz。"
                "在 CPU 紧张的设备上设成 5 可省约 25% CPU"
            ),
        ),
        DeclareLaunchArgument(
            "start_mapper", default_value="true",
            description="是否同时启动语义地图节点",
        ),
    ]

    # ---------------- 静态 TF ----------------
    # base_link -> camera_link 的外参与 ~/VIO/launch_d435_rtabmap.sh 保持一致
    static_base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_to_camera",
        arguments=[
            "--x", "0.045", "--y", "-0.075", "--z", "-0.005",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link", "--child-frame-id", "camera_link",
        ],
        condition=IfCondition(LaunchConfiguration("publish_static_camera_tf")),
        output="log",
    )

    # 仅离线测试使用。实机上 map->base_link 必须由 MAVROS 独占发布，
    # 两个发布者同时存在会让 TF 在两个值之间跳变。
    static_map_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_base",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map", "--child-frame-id", "base_link",
        ],
        condition=IfCondition(LaunchConfiguration("publish_static_map_tf")),
        output="log",
    )

    # ---------------- 语义检测节点（Phase 1）----------------
    detector = Node(
        package="semantic_rtabmap",
        executable="detector_node",
        name="semantic_detector",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "target_frame": LaunchConfiguration("target_frame"),
                "model_path": LaunchConfiguration("model_path"),
                "class_filter": LaunchConfiguration("class_filter"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                "process_rate": LaunchConfiguration("process_rate"),
            },
        ],
        output="screen",
        emulate_tty=True,
    )

    # ---------------- 语义地图节点（Phase 2）----------------
    mapper = Node(
        package="semantic_rtabmap",
        executable="mapper_node",
        name="semantic_mapper",
        parameters=[
            LaunchConfiguration("mapper_params_file"),
            {
                "map_frame": LaunchConfiguration("target_frame"),
                "persistence_path": LaunchConfiguration("persistence_path"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_mapper")),
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        args + [static_map_to_base, static_base_to_camera, detector, mapper]
    )
