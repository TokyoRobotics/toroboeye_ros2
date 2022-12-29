import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            name='node_prefix',
            default_value= '',
            description='Prefix for node names'),

        # toroboeye_camera (server)
        launch_ros.actions.Node(
            package='toroboeye_camera',
            executable='service',
            namespace='toroboeye',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('toroboeye_camera'), 'param_filter.yaml'),
                os.path.join(get_package_share_directory('toroboeye_camera'), 'param_capture_setting.yaml')
            ],
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'service']
        ),

        # For publishing PointCloud2 data with depth_image_proc
        # http://wiki.ros.org/depth_image_proc#depth_image_proc.2Fpoint_cloud_xyzrgb

        # launch_ros.actions.ComposableNodeContainer(
        #     name='container',
        #     namespace='toroboeye',
        #     package='rclcpp_components',
        #     executable='component_container',
        #     composable_node_descriptions=[
        #         # Driver itself
        #         launch_ros.descriptions.ComposableNode(
        #             package='depth_image_proc',
        #             plugin='depth_image_proc::PointCloudXyzrgbNode',
        #             name='point_cloud_xyzrgb_node',
        #             remappings=[
        #                 ("rgb/camera_info", "/toroboeye/camera_info"),
        #                 ("rgb/image_rect_color", "/toroboeye/color_image"),
        #                 ("depth_registered/image_rect", "/toroboeye/depth_image"),
        #                 ("points", "/toroboeye/points")
        #             ],
        #         ),
        #     ],
        #     output='screen',
        # ),

    ])
