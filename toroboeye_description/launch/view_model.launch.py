import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('toroboeye_description'), 'rviz', 'rviz_config.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value='sl80',
            description='model name'
        ),
        
        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            name='urdf',
            default_value=[LaunchConfiguration('model'), '.urdf.xacro'],
            description='urdf filename'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', 
                        PathJoinSubstitution(
                                [FindPackageShare("toroboeye_description"), "urdf", LaunchConfiguration('urdf')]
                            )
                        ])
                }
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
