from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch the MAV UNIFORM PUMP node with configurable parameters
    """

    # Declare launch arguments
    batch_interval_arg = DeclareLaunchArgument(
        'batch_interval',
        default_value='1.0',
        description='Batch interval (dT) in seconds - how often to publish data packages'
    )

    condensation_mode_arg = DeclareLaunchArgument(
        'condensation_mode',
        default_value='raw',
        description='Data condensation mode: raw, multi_step, or single_step'
    )

    multi_step_count_arg = DeclareLaunchArgument(
        'multi_step_count',
        default_value='3',
        description='Number of discrete steps for multi_step mode'
    )

    bleeding_domain_duration_arg = DeclareLaunchArgument(
        'bleeding_domain_duration',
        default_value='15.0',
        description='Duration in seconds to keep historical data for filling missing fields'
    )

    missing_data_strategy_arg = DeclareLaunchArgument(
        'missing_data_strategy',
        default_value='bleeding_average',
        description='Strategy for missing data: bleeding_average, bleeding_latest, null, or zero'
    )

    # Create the node
    uniform_pump_node = Node(
        package='mav_data_hose',
        executable='uniform_pump_node',
        name='uniform_pump',
        output='screen',
        parameters=[{
            'batch_interval': LaunchConfiguration('batch_interval'),
            'condensation_mode': LaunchConfiguration('condensation_mode'),
            'multi_step_count': LaunchConfiguration('multi_step_count'),
            'bleeding_domain_duration': LaunchConfiguration('bleeding_domain_duration'),
            'missing_data_strategy': LaunchConfiguration('missing_data_strategy'),
        }]
    )

    return LaunchDescription([
        batch_interval_arg,
        condensation_mode_arg,
        multi_step_count_arg,
        bleeding_domain_duration_arg,
        missing_data_strategy_arg,
        uniform_pump_node
    ])
