"""
perception.launch.py
Launches the YOLOv8n + ONNX Runtime perception node.

Usage:
  ros2 launch my_robot_controller perception.launch.py
  ros2 launch my_robot_controller perception.launch.py frame_skip:=2 conf_threshold:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("frame_skip",     default_value="1",
                              description="Process every Nth frame (1 = all)"),
        DeclareLaunchArgument("input_width",    default_value="128"),
        DeclareLaunchArgument("input_height",   default_value="128"),
        DeclareLaunchArgument("conf_threshold", default_value="0.40",
                              description="YOLO confidence threshold [0–1]"),
        DeclareLaunchArgument("camera_topic",   default_value="/camera/image_raw"),
        DeclareLaunchArgument("output_topic",   default_value="/detected_objects"),
        DeclareLaunchArgument("num_workers",    default_value="1",
                              description="Parallel model instances (1 avoids CPU contention)"),
    ]

    perception_node = Node(
        package="my_robot_controller",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[{
            "frame_skip":     LaunchConfiguration("frame_skip"),
            "input_width":    LaunchConfiguration("input_width"),
            "input_height":   LaunchConfiguration("input_height"),
            "conf_threshold": LaunchConfiguration("conf_threshold"),
            "camera_topic":   LaunchConfiguration("camera_topic"),
            "output_topic":   LaunchConfiguration("output_topic"),
            "num_workers":    LaunchConfiguration("num_workers"),
        }],
    )

    return LaunchDescription(args + [perception_node])
