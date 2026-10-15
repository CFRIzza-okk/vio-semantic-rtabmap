#!/usr/bin/env python3
"""R680 自主探索启动文件。

只负责“感知 -> 选点 -> 交给 Nav2 -> 安全闸门”这条链路的启动，
不启动 TX1 侧驱动、不启动 relay、也不启动 RTAB-Map 建图：

* TX1 ROS 1 驱动与 relay：在 TX1 上单独启动（见 tools/README_R680_RELAY.md）；
* RTAB-Map 建图与定位：单独启动，它负责发布 /map 与 map->odom。

默认最保守：``dry_run:=true`` 且 ``autostart:=false``，启动后机器人不会动，
只会在 /exploration/goal 上发布它想去的点，方便先验证选点是否合理。

真正放开运动需要三件事同时满足：

    ros2 launch semantic_rtabmap r680_exploration.launch.py dry_run:=false
    ros2 service call /exploration/start std_srvs/srv/Trigger
    安全节点以外没有任何节点直接发布 /cmd_vel
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("semantic_rtabmap")

    params_file = LaunchConfiguration("params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    safety_params_file = LaunchConfiguration("safety_params_file")
    include_nav2 = LaunchConfiguration("include_nav2")
    include_scan = LaunchConfiguration("include_scan")
    include_safety = LaunchConfiguration("include_safety")
    depth_topic = LaunchConfiguration("depth_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    cmd_vel_nav_topic = LaunchConfiguration("cmd_vel_nav_topic")
    autostart = LaunchConfiguration("autostart")
    dry_run = LaunchConfiguration("dry_run")
    max_linear_x = LaunchConfiguration("max_linear_x")
    max_linear_y = LaunchConfiguration("max_linear_y")
    max_angular_z = LaunchConfiguration("max_angular_z")
    use_sim_time = LaunchConfiguration("use_sim_time")

    explorer_params = [
        params_file,
        {
            # 显式声明类型，避免把字符串塞进 bool 参数。
            "autostart": ParameterValue(autostart, value_type=bool),
            "dry_run": ParameterValue(dry_run, value_type=bool),
            # 显式写全名：RTAB-Map 的 namespace 默认是 rtabmap，发在 /rtabmap/map。
            # 写 /map 会让探索器订阅一个不存在的话题，永远选不出 frontier。
            "map_topic": "/rtabmap/map",
        },
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "r680_exploration.yaml"),
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=os.path.join(share, "config", "r680_nav2.yaml"),
        ),
        DeclareLaunchArgument(
            "safety_params_file",
            default_value=os.path.join(share, "config", "r680_navigation.yaml"),
        ),
        DeclareLaunchArgument(
            "include_nav2",
            default_value="true",
            description="是否同时启动 Nav2（关闭时只做选点与安全监视）",
        ),
        DeclareLaunchArgument(
            "include_safety",
            default_value="true",
            description="是否启动 cmd_vel_safety 安全闸门",
        ),
        DeclareLaunchArgument(
            "cmd_vel_nav_topic",
            default_value="/cmd_vel_nav",
            description="安全闸门的输入：Nav2 controller 与 recovery 的输出",
        ),
        DeclareLaunchArgument(
            "max_linear_x", default_value="0.20", description="安全闸门限速 (m/s)"
        ),
        DeclareLaunchArgument(
            "max_linear_y", default_value="0.08", description="安全闸门横移限速 (m/s)"
        ),
        DeclareLaunchArgument(
            "max_angular_z", default_value="0.50", description="安全闸门角速度上限 (rad/s)"
        ),
        DeclareLaunchArgument(
            "include_scan",
            default_value="true",
            description="是否把 D435 对齐深度转成二维 /scan",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/aligned_depth_to_color/camera_info",
            description=(
                "depthimage_to_laserscan 的 camera_info 话题。"
                "必须显式 remap：该节点的默认话题名是 depth_camera_info，"
                "而 D435 从不发布这个名字，所以漏掉这一项时它一直等 camera_info，"
                "每帧都打印 'No camera info, skipping point cloud squash'，"
                "/scan 一帧都不出，进而让 obstacle_monitor 按'障碍存在'处理。"
            ),
        ),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "autostart", default_value="false", description="启动后是否立刻开始探索"
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="true 时只选点不驱动底盘，务必先验证选点",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # D435 对齐深度 -> 二维 LaserScan，供 costmap 与障碍监视器使用。
        Node(
            package="depthimage_to_laserscan",
            executable="depthimage_to_laserscan_node",
            name="depthimage_to_laserscan",
            output="screen",
            condition=IfCondition(include_scan),
            parameters=[params_file, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            remappings=[
                ("depth", depth_topic),
                # 注意 remap 的**源名字**是 depth_camera_info，不是 camera_info。
                # 这个节点自己把订阅建在 depth_camera_info 上（官方
                # depthimage_to_laserscan-launch.py 里同样是
                # ('depth_camera_info', '/camera/depth/camera_info')）。
                # 写成 ("camera_info", ...) 不会生效，也不会报错 —— 节点照旧订阅
                # /depth_camera_info 等一个永远不来的消息，每帧打印
                # "No camera info, skipping point cloud squash"，/scan 无输出。
                ("depth_camera_info", camera_info_topic),
                ("scan", scan_topic),
            ],
        ),

        # 障碍与定位安全输入：安全节点靠这两个话题决定能不能放行速度。
        Node(
            package="semantic_rtabmap",
            executable="obstacle_monitor_node",
            name="obstacle_monitor",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="semantic_rtabmap",
            executable="localization_monitor_node",
            name="localization_monitor",
            output="screen",
            parameters=[params_file],
        ),

        # 探索器：只发 Nav2 目标，永远不直接发速度。
        Node(
            package="semantic_rtabmap",
            executable="exploration_node",
            name="r680_explorer",
            output="screen",
            parameters=explorer_params,
        ),

        # 安全闸门：整条链路里唯一允许写 /cmd_vel_safe 的节点。
        Node(
            package="semantic_rtabmap",
            executable="cmd_vel_safety_node",
            name="cmd_vel_safety",
            output="screen",
            condition=IfCondition(include_safety),
            parameters=[
                safety_params_file,
                {
                    "input_topic": cmd_vel_nav_topic,
                    "output_topic": "/cmd_vel_safe",
                    "max_linear_x": ParameterValue(max_linear_x, value_type=float),
                    "max_linear_y": ParameterValue(max_linear_y, value_type=float),
                    "max_angular_z": ParameterValue(max_angular_z, value_type=float),
                },
            ],
        ),

        # 本工程自己的 Nav2 bringup：controller 与 behavior 的输出都汇到
        # /cmd_vel_nav，再由 cmd_vel_safety 放行。不使用官方
        # navigation_launch.py —— 它会让 behavior_server 和 velocity_smoother
        # 直接发布 /cmd_vel，绕过安全节点。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "r680_nav2_bringup.launch.py")
            ),
            condition=IfCondition(include_nav2),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params_file,
                "autostart": "true",
            }.items(),
        ),
    ])
