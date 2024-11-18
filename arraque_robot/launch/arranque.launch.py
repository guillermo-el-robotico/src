#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define el nodo para control serial
    control_serial_node = Node(
        package='robot_motor',
        executable='control_serial.py',
        name='control_serial',
        output='screen'
    )

    # Define el nodo para odometría y batería
    odom_battery_node = Node(
        package='robot_odometry',
        executable='odom_publisher_battery.py',
        name='odom_publisher_battery',
        output='screen'
    )

    # Incluir el archivo launch del robot (display.launch.py)
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_cuerpo'),
                'launch',
                'display.launch.py'
            )
        )
    )

    # Incluir el archivo launch para el LiDAR (sllidar_s2_launch.py)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_s2_launch.py'
            )
        )
    )

    # Incluir el archivo launch para slam_toolbox (online_async_launch.py)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        )
    )

    return LaunchDescription([
        control_serial_node,
        odom_battery_node,
        display_launch,
        lidar_launch,
        slam_launch
    ])
