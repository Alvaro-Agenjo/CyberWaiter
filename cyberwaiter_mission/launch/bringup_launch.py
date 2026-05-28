from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments (package and executable names are configurable)
    kin_pkg = LaunchConfiguration('kin_pkg')
    kin_exec = LaunchConfiguration('kin_exec')

    nav_pkg = LaunchConfiguration('nav_pkg')
    nav_exec = LaunchConfiguration('nav_exec')

    gripper_pkg = LaunchConfiguration('gripper_pkg')
    gripper_exec = LaunchConfiguration('gripper_exec')

    mission_pkg = LaunchConfiguration('mission_pkg')
    mission_exec = LaunchConfiguration('mission_exec')

    declared_args = [
        DeclareLaunchArgument('kin_pkg', default_value='kin_silver', description='Package for kin node'),
        DeclareLaunchArgument('kin_exec', default_value='kin_silver', description='Executable name for kin node'),

        DeclareLaunchArgument('nav_pkg', default_value='navigator', description='Package for navigator node'),
        DeclareLaunchArgument('nav_exec', default_value='navigator', description='Executable name for navigator node'),

        DeclareLaunchArgument('gripper_pkg', default_value='gripper', description='Package for gripper node'),
        DeclareLaunchArgument('gripper_exec', default_value='gripper', description='Executable name for gripper node'),

        DeclareLaunchArgument('mission_pkg', default_value='cyberwaiter_mission', description='Package for mission node'),
        DeclareLaunchArgument('mission_exec', default_value='mission', description='Executable name for mission node'),
    ]

    kin_node = Node(
        package=kin_pkg,
        executable=kin_exec,
        name='kin_silver',
        output='screen',
    )

    nav_node = Node(
        package=nav_pkg,
        executable=nav_exec,
        name='navigator',
        output='screen',
    )

    gripper_node = Node(
        package=gripper_pkg,
        executable=gripper_exec,
        name='gripper',
        output='screen',
    )

    mission_node = Node(
        package=mission_pkg,
        executable=mission_exec,
        name='mission',
        output='screen',
    )

    ld = LaunchDescription()
    for a in declared_args:
        ld.add_action(a)

    # Add nodes
    ld.add_action(kin_node)
    ld.add_action(nav_node)
    ld.add_action(gripper_node)
    ld.add_action(mission_node)

    return ld
