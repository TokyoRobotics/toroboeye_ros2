import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            name='node_prefix',
            default_value= '',
            description='Prefix for node names'),

        # toroboeye_client (client)
        launch_ros.actions.Node(
            package='toroboeye_camera',
            executable='client',
            namespace='toroboeye',
            output='screen',
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'client']
        ),

    ])
