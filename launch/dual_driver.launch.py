from pathlib import Path
import tempfile
import launch
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler,
                            EmitEvent, LogInfo, OpaqueFunction)
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition


def create_driver(ns, params_files):
    driver = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name='os_driver',
        namespace=ns,
        parameters=params_files,
        output='screen',
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver, goal_state='inactive',
            entities=[
                LogInfo(msg=f"[{ns}] os_driver activating..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        )
    )

    finalized = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver, goal_state='finalized',
            entities=[
                LogInfo(msg=f"[{ns}] Failed to communicate with the sensor."),
            ],
        )
    )

    return [driver, configure, activate, finalized]


def write_azimuth_override(ns, az_start, az_end):
    f = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix=f'{ns}_azimuth_')
    f.write(
        f"{ns}/os_driver:\n"
        f"  ros__parameters:\n"
        f"    azimuth_window_start: {az_start}\n"
        f"    azimuth_window_end: {az_end}\n"
    )
    f.close()
    return f.name


def launch_setup(context):
    pkg_dir = get_package_share_directory('ouster_multi')
    config = Path(pkg_dir) / 'config'

    front_params = str(config / 'driver_params_front.yaml')
    rear_params = str(config / 'driver_params_rear.yaml')

    front_az_start = int(context.launch_configurations['front_azimuth_start'])
    front_az_end = int(context.launch_configurations['front_azimuth_end'])
    rear_az_start = int(context.launch_configurations['rear_azimuth_start'])
    rear_az_end = int(context.launch_configurations['rear_azimuth_end'])

    front_override = write_azimuth_override('ouster_front', front_az_start, front_az_end)
    rear_override = write_azimuth_override('ouster_rear', rear_az_start, rear_az_end)

    return (
        create_driver('ouster_front', [front_params, front_override]) +
        create_driver('ouster_rear', [rear_params, rear_override])
    )


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('front_azimuth_start', default_value='0'),
        DeclareLaunchArgument('front_azimuth_end', default_value='270000'),
        DeclareLaunchArgument('rear_azimuth_start', default_value='0'),
        DeclareLaunchArgument('rear_azimuth_end', default_value='270000'),
        OpaqueFunction(function=launch_setup),
    ])
