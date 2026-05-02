from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    #
    # data_files tells colcon where to install non-Python assets.
    # Both lines below are REQUIRED — without them `ros2 pkg list`
    # cannot find this package after building.
    #
    data_files=[
        # (1) Registers the package with ament's resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # (2) Installs package.xml so rosdep and ros2 pkg info work
        ('share/' + package_name, ['package.xml']),
        # (3) Install launch files
        ('share/' + package_name + '/launch', [
            'launch/gazebo_panda.launch.py',
            'launch/perception.launch.py',
            'launch/drone_tracking.launch.py',
        ]),
        # (4) Install controller config and Gazebo models
        ('share/' + package_name + '/config', [
            'config/panda_ros2_controllers.yaml',
            'config/work_table.sdf',
            'config/simple_camera.sdf',
            'config/coke_can_simple.sdf',
        ]),
        # (5) Install node executables where ros2 run expects them
        ('lib/' + package_name, [
            'scripts/week1_test_node',
            'scripts/perception_node',
            'scripts/tracking_node',
            'scripts/control_node',
            'scripts/stress_test.py',
            'scripts/test_image_pub.py',
            'scripts/latency_validator.py',
            'scripts/generate_plots.py',
            'scripts/validate_week3.sh',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Week 1 motion planning package for Franka Panda via MoveIt 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        #
        # console_scripts maps a terminal command name → Python function.
        # Format: '<command> = <package>.<module>:<function>'
        #
        # After `colcon build`, running:
        #   ros2 run my_robot_controller week1_test_node
        # is exactly equivalent to calling main() in week1_test_node.py
        #
        'console_scripts': [
            'week1_test_node  = my_robot_controller.week1_test_node:main',
            'perception_node  = my_robot_controller.perception_node:main',
            'tracking_node    = my_robot_controller.tracking_node:main',
            'control_node     = my_robot_controller.control_node:main',
            'logger_node      = my_robot_controller.logger_node:main',
        ],
    },
)
