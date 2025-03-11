import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(get_package_share_directory(
        "rtcrobot_nav350"), 'config', 'param.yaml')

    # container = ComposableNodeContainer(
    #     name='nav350_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='rtcrobot_nav350',
    #             plugin='rtcrobot_nav350::Nav350Node',
    #             name='nav350_node',
    #             parameters=[{"ip_address": "192.168.5.98"}],
    #             extra_arguments=[{'use_intra_process_comms': True}]),
    #     ],
    #     output='both',
    # )

    nav350_nodde = Node(
        package='rtcrobot_nav350',
        executable='nav350_node',
        name='nav350_node',
        output='screen',
        parameters=[param_file]
    )

    return launch.LaunchDescription([nav350_nodde])