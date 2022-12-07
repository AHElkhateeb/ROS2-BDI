import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    AGENT_ID = 'tetrabotC3PO'
    AGENT_GROUP_ID = 'transporting_agents'

    debug = False

    namespace = LaunchConfiguration('namespace', default=AGENT_ID)

    planning_mode = 'offline'

    communication_node_params = [
        {'agent_id': AGENT_ID},
        {'agent_group': AGENT_GROUP_ID},
        {'debug': debug},
        {'planning_mode': planning_mode}
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='namespace for launching this robot'),
        Node(
            package='ros2_bdi_core',
            executable='acl_communicator',
            name='acl_communicator',
            namespace=namespace,
            output='screen',
            parameters=communication_node_params
            )
    ])