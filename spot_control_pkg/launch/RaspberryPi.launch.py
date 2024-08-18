#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # Load the URDF into a parameter
    pkgPath = get_package_share_directory('spot_contorl_pkg')
    # pkgPath = launch_ros.substitutions.FindPackageShare(package='test_pkg').find('test_pkg')
    urdf_path = os.path.join(pkgPath, 'urdf', 'Spotmicro.urdf')
    urdf = open(urdf_path, 'r').read()
    params = {'robot_description': urdf}


    robot_state_publisher_node=launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        remappings=[('joint_states', 'spot_joints')],
        output='screen',
        namespace='feedback',
        parameters=[params, {'tf_prefix': 'feedback'}],
        arguments=[urdf_path]
    )

    micro_ros_node = launch_ros.actions.Node(\
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agnet',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '230400'],
        output='screen'
    )

    spot_control_node = launch_ros.actions.Node(\
        package='spot_control_pkg',
        executable='spot_control_node',
        name='spot_control_node'
    ])
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True', description='this'),
        micro_ros_node,
        robot_state_publisher_node,
        spot_control_node
    ])


def main(argv=sys.argv[1:]):
    """Run lifecycle nodes via launch."""
    ld = generate_launch_description()
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()