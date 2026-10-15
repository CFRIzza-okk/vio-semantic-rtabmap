#!/usr/bin/env python3
"""启动 D435 并打开点云，配合只读观测节点。

这是**观测阶段**的启动文件，刻意不碰 Nav2：

* 只启动相机（打开点云）和 ``point_cloud_inspector``；
* 不启动 costmap、不启动 Nav2、不启动探索器、不发布任何速度；
* 目的是先用真实数据回答"地面在相机坐标系里的哪一层、外参是多少"，
  再决定 ``min_obstacle_height`` / ``max_obstacle_height``。

前提：机器人静止、放在平地上、前方 1~3 米有可见地面。

用法::

    ros2 launch semantic_rtabmap r680_depth_cloud.launch.py
    ros2 topic echo /depth_cloud/report

关于点云参数名（踩过的坑）
--------------------------
realsense2_camera **不是**用固定的 ``pointcloud.enable`` 来开关点云的。
它把参数前缀取自 librealsense 的**处理块名字**，并把非字母数字字符
各自替换成一个下划线：

* 这台 Jetson（aarch64）上处理块叫 ``Pointcloud (NEON)``
  → 前缀 ``pointcloud__neon_``（空格→``_``，``(``→``_``，``)``→``_``）
* x86 上处理块叫 ``Pointcloud (SSE3)`` → 前缀 ``pointcloud__sse3_``

所以 ``pointcloud.enable`` 传下去会被**静默忽略**：节点照常启动、
话题也照常出现在 ``topic list`` 里，但**一个发布者都不会创建**，
而且不报错。换一台机器请先用下面的命令查真实前缀：

    ros2 param list /camera/camera | grep -i point

然后用 ``pointcloud_param_prefix:=<前缀>`` 覆盖本文件里的默认值。

注意 ``launch_d435_rtabmap.sh`` 也会启动相机；两者同时跑会因为
USB 带宽 / 设备占用而失败，**同一时间只用一个**。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera(context):
    """Build the camera node with a machine-dependent pointcloud prefix."""
    prefix = LaunchConfiguration("pointcloud_param_prefix").perform(context)
    color_profile = LaunchConfiguration("color_profile").perform(context).strip()
    depth_profile = LaunchConfiguration("depth_profile").perform(context).strip()

    params = {
        "enable_color": True,
        "enable_depth": True,
        "align_depth.enable": True,
        "enable_sync": True,
    }
    # profile **默认不传**：实测在这台机器上强制 depth_module.depth_profile
    # 和 rgb_camera.color_profile（例如 640,480,30）会让相机直接帧超时
    # （librealsense 报 "Frames didn't arrived within 5 seconds"），
    # 点云一帧都出不来；用相机自己的默认 profile（深度 848x480）则完全正常。
    # 所以只有显式给了非空值才覆盖，否则保持默认。
    if color_profile:
        params["rgb_camera.color_profile"] = color_profile
    if depth_profile:
        params["depth_module.depth_profile"] = depth_profile
    # 用查到的真实前缀拼参数名，而不是想当然的 pointcloud.enable。
    params["%s.enable" % prefix] = True
    params["%s.stream_filter" % prefix] = 2      # 2 = 用彩色流给点云上色
    params["%s.ordered_pc" % prefix] = False

    return [
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace="camera",
            output="screen",
            parameters=[params],
        )
    ]


def _inspector(context):
    return [
        Node(
            package="semantic_rtabmap",
            executable="point_cloud_inspector_node",
            name="point_cloud_inspector",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_inspector")),
            parameters=[
                {
                    "cloud_topic": LaunchConfiguration("cloud_topic").perform(context),
                    "report_period": float(
                        LaunchConfiguration("report_period").perform(context)
                    ),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/camera/camera/depth/color/points",
            description="D435 点云话题（相机直出，非 RTAB-Map 的累积地图点云）",
        ),
        DeclareLaunchArgument(
            "pointcloud_param_prefix",
            default_value="pointcloud__neon_",
            description="librealsense 处理块名推导出的参数前缀；"
                        "用 ros2 param list /camera/camera | grep -i point 查证",
        ),
        DeclareLaunchArgument(
            "color_profile",
            default_value="",
            description="留空 = 用相机默认；填 W,H,FPS 会覆盖。注意实测强制 "
                        "640,480,30 会导致帧超时、点云无输出",
        ),
        DeclareLaunchArgument(
            "depth_profile",
            default_value="",
            description="留空 = 用相机默认（848x480x30）；见 color_profile 的警告",
        ),
        DeclareLaunchArgument("report_period", default_value="5.0"),
        DeclareLaunchArgument("start_inspector", default_value="true"),

        OpaqueFunction(function=_camera),
        OpaqueFunction(function=_inspector),
    ])
