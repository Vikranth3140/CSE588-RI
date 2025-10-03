from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'True'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/vikranth/turtlebot3_ws/src/vikranth_gazebo/maps/map_house.yaml',
            description='Full path to map yaml file'
        ),
        nav2_launch,
        Node(
            package='vikranth_gazebo',
            executable='spa_nav',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])