from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    server_arg = DeclareLaunchArgument(
        'server',
        default_value='192.168.0.103',
        description='VRPN server address')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='3883',
        description='VRPN server port')

    vrpn_node = Node(
        package='vrpn_mocap',
        namespace='vrpn_mocap',
        executable='client_node',
        name='vrpn_mocap_client_node',
        parameters=[
            '/home/ginko/dipl_ws/src/astro/config/client.yaml',
            {'server': LaunchConfiguration('server')},
            {'port': LaunchConfiguration('port')}
        ]
    )

    # Add a static transform broadcaster node
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world',
        arguments=["0", "0", "0", "0", "0", "0", "1", "/world", "/odom"] # x, y, z, qx, qy, qz, qw, parent_frame, child_frame
    )

    return LaunchDescription([
        server_arg,
        port_arg,
        vrpn_node,
        static_tf
    ])

