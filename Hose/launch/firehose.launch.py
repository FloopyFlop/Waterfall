from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch the MAV FIRE HOSE node with configurable parameters
    """

    # Declare launch arguments
    connection_string_arg = DeclareLaunchArgument(
        'connection_string',
        default_value='udp:127.0.0.1:14550',
        description='MAVLink connection string (e.g., udp:127.0.0.1:14550, tcp:127.0.0.1:5760, /dev/ttyACM0)'
    )

    data_stream_rate_arg = DeclareLaunchArgument(
        'data_stream_rate',
        default_value='100',
        description='Requested data stream rate in Hz (higher = more data)'
    )

    publish_raw_bytes_arg = DeclareLaunchArgument(
        'publish_raw_bytes',
        default_value='true',
        description='Publish raw MAVLink bytes (true/false)'
    )

    # Create the node
    firehose_node = Node(
        package='mav_data_hose',
        executable='firehose_node',
        name='mav_firehose',
        output='screen',
        parameters=[{
            'connection_string': LaunchConfiguration('connection_string'),
            'data_stream_rate': LaunchConfiguration('data_stream_rate'),
            'publish_raw_bytes': LaunchConfiguration('publish_raw_bytes'),
        }]
    )

    return LaunchDescription([
        connection_string_arg,
        data_stream_rate_arg,
        publish_raw_bytes_arg,
        firehose_node
    ])
