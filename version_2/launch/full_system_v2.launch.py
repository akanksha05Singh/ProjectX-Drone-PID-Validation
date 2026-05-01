"""
full_system_v2.launch.py — Week 4 Complete Stack Launcher
==========================================================
Reformerz Healthcare | Drone Tracking Pipeline v2

Starts all ROS 2 nodes in a single command:
  1. perception_node   — YOLOv8n ONNX 128×128 inference
  2. scenario_runner   — test scenario middleware (default: normal)
  3. tracking_node     — reads from /detected_objects_filtered
  4. control_node_v2   — PID controller with 4-state FSM
  5. logger_v2         — CSV flight recorder

NOTE: PX4 SITL, MAVROS, and test_image_pub must be started separately.

Usage:
  ros2 launch my_robot_controller full_system_v2.launch.py

  # With a test scenario:
  ros2 launch my_robot_controller full_system_v2.launch.py scenario:=occlusion

  # Tune PID gains:
  ros2 launch my_robot_controller full_system_v2.launch.py \\
      kp_yaw:=0.6 ki_yaw:=0.06 kd_yaw:=0.10
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Declare all tunable arguments ────────────────────────────────────────
    args = [
        # Scenario
        DeclareLaunchArgument("scenario",      default_value="normal",
                              description="Test scenario: normal|flicker|occlusion|high_error"),
        DeclareLaunchArgument("error_offset_x", default_value="0.3",
                              description="high_error: normalised x offset"),

        # PID gains — yaw axis
        DeclareLaunchArgument("kp_yaw",        default_value="0.5"),
        DeclareLaunchArgument("ki_yaw",        default_value="0.05"),
        DeclareLaunchArgument("kd_yaw",        default_value="0.08"),

        # PID gains — forward velocity axis
        DeclareLaunchArgument("kp_vx",         default_value="0.4"),
        DeclareLaunchArgument("ki_vx",         default_value="0.04"),
        DeclareLaunchArgument("kd_vx",         default_value="0.06"),

        # Saturation limits
        DeclareLaunchArgument("max_yaw",       default_value="0.6"),
        DeclareLaunchArgument("max_vx",        default_value="0.5"),
        DeclareLaunchArgument("deadband",      default_value="0.05"),
        DeclareLaunchArgument("search_timeout",default_value="1.5",
                              description="Seconds before SEARCH→HOLD transition"),

        # Perception
        DeclareLaunchArgument("image_width",   default_value="128"),
        DeclareLaunchArgument("image_height",  default_value="128"),
        DeclareLaunchArgument("conf_threshold",default_value="0.40"),

        # Logger
        DeclareLaunchArgument("session_prefix",default_value="session_v2"),
    ]

    # ── 1. Perception node (unchanged from Week 2/3) ─────────────────────────
    perception_node = Node(
        package="my_robot_controller",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[{
            "frame_skip":      1,
            "input_width":     LaunchConfiguration("image_width"),
            "input_height":    LaunchConfiguration("image_height"),
            "conf_threshold":  LaunchConfiguration("conf_threshold"),
            "camera_topic":    "/camera/image_raw",
            "output_topic":    "/detected_objects",
        }],
    )

    # ── 2. Scenario runner (middleware) ───────────────────────────────────────
    scenario_runner = Node(
        package="my_robot_controller",
        executable="scenario_runner",
        name="scenario_runner",
        output="screen",
        parameters=[{
            "scenario":           LaunchConfiguration("scenario"),
            "flicker_prob":       0.30,
            "occlusion_period":   5.0,
            "occlusion_duration": 2.0,
            "error_offset_x":     LaunchConfiguration("error_offset_x"),
            "error_offset_y":     0.0,
            "image_width":        LaunchConfiguration("image_width"),
            "image_height":       LaunchConfiguration("image_height"),
        }],
    )

    # ── 3. Tracking node — reads filtered detections ─────────────────────────
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
            "input_topic":   "/detected_objects_filtered",   # <── filtered
        }],
    )

    # ── 4. Control node v2 — PID ─────────────────────────────────────────────
    control_node_v2 = Node(
        package="my_robot_controller",
        executable="control_node_v2",
        name="control_node_v2",
        output="screen",
        parameters=[{
            "kp_yaw":         LaunchConfiguration("kp_yaw"),
            "ki_yaw":         LaunchConfiguration("ki_yaw"),
            "kd_yaw":         LaunchConfiguration("kd_yaw"),
            "kp_vx":          LaunchConfiguration("kp_vx"),
            "ki_vx":          LaunchConfiguration("ki_vx"),
            "kd_vx":          LaunchConfiguration("kd_vx"),
            "max_yaw":        LaunchConfiguration("max_yaw"),
            "max_vx":         LaunchConfiguration("max_vx"),
            "deadband":       LaunchConfiguration("deadband"),
            "search_timeout": LaunchConfiguration("search_timeout"),
            "integral_limit": 2.0,
        }],
    )

    # ── 5. Logger v2 ─────────────────────────────────────────────────────────
    logger_v2 = Node(
        package="my_robot_controller",
        executable="logger_node",       # registered in setup.py entry_points
        name="logger_v2",
        output="screen",
        parameters=[{
            "session_prefix":       LaunchConfiguration("session_prefix"),
            "flush_interval_rows":  100,
        }],
    )

    return LaunchDescription(args + [
        perception_node,
        scenario_runner,
        tracking_node,
        control_node_v2,
        logger_v2,
    ])
