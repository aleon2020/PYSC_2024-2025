import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('exploration_example')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/exploration_domain_example.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    move_cmd = Node(
        package='exploration_example',
        executable='move_action_node',
        name='move_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    collect_cmd = Node(
        package='exploration_example',
        executable='collect_action_node',
        name='collect_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    drop_cmd = Node(
        package='exploration_example',
        executable='drop_action_node',
        name='drop_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    analyse_soil_cmd = Node(
        package='exploration_example',
        executable='analyse_soil_action_node',
        name='analyse_soil_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_cmd)
    ld.add_action(collect_cmd)
    ld.add_action(drop_cmd)
    ld.add_action(analyse_soil_cmd)

    return ld
