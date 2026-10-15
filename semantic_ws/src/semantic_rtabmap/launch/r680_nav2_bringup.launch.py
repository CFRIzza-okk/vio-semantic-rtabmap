#!/usr/bin/env python3
"""R680 专用 Nav2 启动文件（不使用 nav2_bringup 的 navigation_launch.py）。

为什么不直接用官方 navigation_launch.py
--------------------------------------
实测（``ros2 node info``）官方 bringup 的速度话题接线是：

* ``controller_server``  发布 ``/cmd_vel_nav``
* ``behavior_server``    发布 ``/cmd_vel``        <-- 没有经过安全节点
* ``velocity_smoother``  发布 ``/cmd_vel``        <-- 没有经过安全节点

也就是说 Nav2 的最终速度输出落在 ``/cmd_vel`` 上，绕过了
``cmd_vel_safety``（它只拦 ``/cmd_vel_nav``）。恢复动作和"Smoother 之后"
的速度都属于无人把关的状态，这对一台能自己跑的小车是不可接受的。

本启动文件把速度通路收敛成唯一一条：

    controller_server ─┐
    behavior_server  ─┴─→ /cmd_vel_nav → cmd_vel_safety → /cmd_vel_safe → relay

具体做法：

1. 给 ``controller_server`` **和** ``behavior_server`` 都加
   ``cmd_vel -> cmd_vel_nav`` 的重映射，恢复动作同样受安全节点管辖；
2. **不启动 velocity_smoother** —— 多一个速度发布者就多一条绕过安全节点的路径。
   若将来确实需要平滑，必须把它插在 ``cmd_vel_safety`` 之前
   （输入 ``/cmd_vel_nav``、输出到安全节点的输入话题），而不是接在它后面。
3. 不启动 amcl、不启动 map_server：定位与地图由 RTAB-Map 提供。

安全属性：本文件结束时，整个系统里唯一发布 ``/cmd_vel_safe`` 的节点是
``cmd_vel_safety``；没有任何节点发布 ``/cmd_vel``。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = get_package_share_directory("semantic_rtabmap")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    # 只为需要的节点生成参数：把 use_sim_time 统一改写，
    # 并且只保留 Nav2 相关段落，避免把探索器/监视器的参数喂给 Nav2 节点。
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # 唯一的“底盘速度输入”话题。controller 和 recovery 都汇到这里。
    cmd_vel_nav_remap = ("cmd_vel", "cmd_vel_nav")
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_params],
        # 控制器输出进入安全链路，而不是直接进 /cmd_vel。
        remappings=tf_remaps + [cmd_vel_nav_remap],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_params],
        # 关键修正：恢复动作（spin / backup / drive_on_heading）同样必须受
        # cmd_vel_safety 管辖，否则它们会直接写 /cmd_vel。
        remappings=tf_remaps + [cmd_vel_nav_remap],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_params],
        remappings=tf_remaps,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
        remappings=tf_remaps,
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[configured_params],
        remappings=tf_remaps,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                ]
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "r680_nav2.yaml"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        controller_server,
        behavior_server,
        planner_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])
