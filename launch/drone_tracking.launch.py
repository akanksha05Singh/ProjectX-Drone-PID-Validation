"""
Week 3 — Drone Tracking Launch File  |  Reformerz Healthcare Robotics
======================================================================
Starts the perception + tracking + control pipeline for PX4 SITL.

NOTE: This launch file starts ONLY the ROS 2 nodes.
      Gazebo PX4 SITL and MAVROS must be started separately (see README).

Usage:
  ros2 launch my_robot_controller drone_tracking.launch.py
  ros2 launch my_robot_controller drone_tracking.launch.py kp_yaw:=0.8
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Tunable gains (pass at launch time without recompiling) ─────────────
    args = [
        DeclareLaunchArgument("kp_yaw",      default_value="0.5",
                              description="Yaw P-gain (error.x → yaw_rate)"),
        DeclareLaunchArgument("kp_vx",       default_value="0.4",
                              description="Forward P-gain (error.y → vx)"),
        DeclareLaunchArgument("max_yaw",     default_value="0.6",
                              description="Max yaw_rate saturation (rad/s)"),
        DeclareLaunchArgument("max_vx",      default_value="0.5",
                              description="Max forward speed saturation (m/s)"),
        DeclareLaunchArgument("deadband",    default_value="0.05",
                              description="Error deadband (normalised units)"),
        DeclareLaunchArgument("image_width", default_value="128",
                              description="Perception model input width (px)"),
        DeclareLaunchArgument("image_height",default_value="128",
                              description="Perception model input height (px)"),
    ]

    # ── Perception node (Week 2, unchanged) ──────────────────────────────────
    perception_node = Node(
        package="my_robot_controller",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[{
            "frame_skip":      1,
            "input_width":     128,
            "input_height":    128,
            "conf_threshold":  0.40,
            "camera_topic":    "/camera/image_raw",
            "output_topic":    "/detected_objects",
        }],
    )

    # ── Tracking node (Week 3 — NEW) ─────────────────────────────────────────
    tracking_node = Node(
        package="my_robot_controller",
        executable="tracking_node",
        name="tracking_node",
        output="screen",
        parameters=[{
            "image_width":   LaunchConfiguration("image_width"),
            "image_height":  LaunchConfiguration("image_height"),
            "target_class":  "person",
            "lost_timeout":  0.5,
            "input_topic":   "/detected_objects",
        }],
    )

    # ── Control node (Week 3 — NEW) ──────────────────────────────────────────
    control_node = Node(
        package="my_robot_controller",
        executable="control_node",
        name="control_node",
        output="screen",
        parameters=[{
            "kp_yaw":      LaunchConfiguration("kp_yaw"),
            "kp_vx":       LaunchConfiguration("kp_vx"),
            "max_yaw":     LaunchConfiguration("max_yaw"),
            "max_vx":      LaunchConfiguration("max_vx"),
            "deadband":    LaunchConfiguration("deadband"),
            "takeoff_alt": 2.0,
        }],
    )

    return LaunchDescription(args + [
        perception_node,
        tracking_node,
        control_node,
    ])
