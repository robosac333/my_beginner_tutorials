from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declaring arguments
    freq_arg = DeclareLaunchArgument(
        'frequency',
        default_value='2.0',
        description='Publishing frequency in Hz'
    )

    # Creating nodes
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='talker',
        parameters=[{
            'frequency': LaunchConfiguration('frequency')
        }],
        output='screen'
    )

    listener_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='listener',
        output='screen'
    )

    return LaunchDescription([
        freq_arg,
        talker_node,
        listener_node
    ])
