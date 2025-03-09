import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
from time import time

def generate_launch_description():
    # Get the path to the RViz config file
    rviz_config_path = os.path.join(
        get_package_share_directory('object_tracking'), 'rviz2', 'object_tracking.rviz'
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
        # Wait 2 seconds until RViz2 is ready, 
        # otherwise the robot path will not be displayed
        TimerAction(
            period=2.0,
            actions=[
              LogInfo(msg="RViz2 is ready, launching other nodes."),
        Node(
            package='object_tracking',
            executable='object_tracker',
            name='object_tracker',
            output='screen'
        ),
        Node(
            package='object_tracking',
            executable='transform_detections',
            name='transform_detections',
            output='screen'
        ),
        Node(
            package='object_tracking',
            executable='sensor_data_publisher',
            name='sensor_data_publisher',
            output='screen'
        )
            ]
        )
    ])