from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from datetime import datetime
import os

def generate_launch_description():
    # Current timestamp for the bag file name
    timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')

    # Defining the bag file path
    bag_path = os.path.join(
        os.getcwd(),
        'src',
        'my_beginner_tutorials',
        'results',
        'ros2_assg_3_imgs',
        f'rosbag2_{timestamp}'
    )

    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(bag_path), exist_ok=True)

    # Using timeout command to limit recording to 15 seconds
    record_cmd = [
        'timeout', '--preserve-status', '15',
        'ros2', 'bag', 'record', '-a',
        '--output', bag_path
    ]

    # Creates the recording process
    recorder = ExecuteProcess(
        cmd=record_cmd,
        output='screen'
    )

    # Creates shutdown event
    shutdown_event = EmitEvent(
        event=Shutdown(
            reason='Recording completed'
        )
    )

    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder,
            on_exit=[shutdown_event]
        )
    )

    return LaunchDescription([
        recorder,
        exit_handler
    ])
