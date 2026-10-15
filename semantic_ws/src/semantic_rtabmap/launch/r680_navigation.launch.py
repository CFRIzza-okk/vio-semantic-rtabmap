#!/usr/bin/env python3
"""R680 ROS 2 integration launch.

The TX1 ROS 1 side is intentionally not started here: launch the verified
Wheeltec driver and the custom UDP relay on TX1 (ros1_bridge is not available
in either image). This launch owns the R680-specific static TF and the single
ROS 2 safety gate.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("publish_base_tf", default_value="true"),
        DeclareLaunchArgument("publish_camera_tf", default_value="true"),
        # Camera mount, measured 2026-10-13 on the real robot.
        #
        # z / pitch come from a floor-plane fit (point_cloud_inspector_node).
        # Two independent live collections agree with each other far better than
        # with the first, cruder number:
        #   90 s / n=16 : 0.1639 +- 0.0027 m,  1.60 +- 0.29 deg down
        #   120 s / n=20: 0.1648 +- 0.0027 m,  1.66 +- 0.26 deg down
        # The two sets differ by 0.9 mm and 0.06 deg, so the real repeatability is
        # much better than the per-frame scatter suggests.  Both sit ~5 mm below
        # the earlier 0.17 m estimate, which was biased high: the first estimator
        # averaged the top decile of y per slice, and averaging a truncated tail
        # sits 1.755*sigma_y above the truth (+7.0 mm at sigma=4 mm, measured).
        # The values below are the rounded consensus of the two sets.
        #
        # base_link z=0 *is* the ground plane (per mini_mec_robot.urdf the wheels
        # are r=0.0375 centred at z=0.0375), so the measured height is camera_z
        # directly -- no offset.
        #
        # x comes from a tape measure: the D435 lens sits ~15 cm forward of the
        # wheel-rectangle centre.  That centre IS base_link's origin: the URDF
        # places the four wheels symmetrically at x=+-0.085, y=+-0.0975, and TX1
        # integrates the lower board's chassis velocity with no lever-arm term,
        # so odom (and therefore base_link) rotates about exactly that point.
        #
        # y/roll/yaw are ASSUMED zero (camera on the centreline, square to the
        # chassis).  They were not measured.  A wrong y shows up as map smear
        # when the robot rotates in place -- check it during the first in-place
        # rotation, and refine with the calibration board if it is visible.
        DeclareLaunchArgument("camera_x", default_value="0.15"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.165"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        # +0.02897 rad = 1.66 deg.  Positive pitch means "looking down" in both
        # conventions involved here: R_y(+) tilts the forward axis toward -z in
        # REP-103, and the inspector reports estimated_pitch_deg_down as positive
        # when the camera looks down.  Keep them agreeing.
        DeclareLaunchArgument("camera_pitch", default_value="0.02897"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        DeclareLaunchArgument("max_linear_x", default_value="0.35"),
        DeclareLaunchArgument("max_linear_y", default_value="0.35"),
        DeclareLaunchArgument("max_angular_z", default_value="0.8"),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="r680_base_to_footprint",
            arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_footprint", "--child-frame-id", "base_link"],
            condition=IfCondition(LaunchConfiguration("publish_base_tf")), output="screen"),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="r680_base_to_camera",
            arguments=["--x", LaunchConfiguration("camera_x"), "--y", LaunchConfiguration("camera_y"),
                       "--z", LaunchConfiguration("camera_z"), "--roll", LaunchConfiguration("camera_roll"),
                       "--pitch", LaunchConfiguration("camera_pitch"), "--yaw", LaunchConfiguration("camera_yaw"),
                       "--frame-id", "base_link", "--child-frame-id", "camera_link"],
            condition=IfCondition(LaunchConfiguration("publish_camera_tf")), output="screen"),
        Node(
            package="semantic_rtabmap", executable="cmd_vel_safety_node",
            name="cmd_vel_safety",
            parameters=[{
                "input_topic": "/cmd_vel_nav", "output_topic": "/cmd_vel_safe",
                "max_linear_x": LaunchConfiguration("max_linear_x"),
                "max_linear_y": LaunchConfiguration("max_linear_y"),
                "max_angular_z": LaunchConfiguration("max_angular_z"),
            }], output="screen"),
    ])
