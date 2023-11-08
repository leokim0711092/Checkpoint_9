import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():

    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::PreApproach',
                    name='PreApproach'),
            ],
            output='screen',
    )

    # container2 = ComposableNodeContainer(
    #         name='my_container',
    #         namespace='',
    #         package='rclcpp_components',
    #         executable='component_container',
    #         composable_node_descriptions=[
    #             ComposableNode(
    #                 package='my_components',
    #                 plugin='my_components::attach_server',
    #                 name='attach_server'),
    #         ],
    #         output='screen',
    # )
    attach_server = Node(
        package='my_components',
        executable='attach_server',
        output='screen',
        emulate_tty=True
        )



    return launch.LaunchDescription([
    container,
    attach_server
    ])