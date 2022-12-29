import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value='sl80',
            description='model name'
        ),

        # toroboeye_camera (server)
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource([
                launch_ros.substitutions.FindPackageShare("toroboeye_camera"), "/toroboeye_camera.launch.py"
            ]),
        ),

        # [rviz] include toroboeye_description
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource([
                launch_ros.substitutions.FindPackageShare("toroboeye_description"), "/launch/view_model.launch.py"
            ]),
            launch_arguments={
                "model": launch.substitutions.LaunchConfiguration('model'),
            }.items()
        ),

    ])
