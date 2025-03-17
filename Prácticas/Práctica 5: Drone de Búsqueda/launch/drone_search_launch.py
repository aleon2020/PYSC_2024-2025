import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('drone_search')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': pkg_dir + '/pddl/domain.pddl'}.items()
    )
    # Specify the actions
    fly_cmd = Node(
        package='drone_search',
        executable='fly_action_node',
        name='fly_action_node',
        output='screen',
        parameters=[])
    
    search_cmd = Node(
        package='drone_search',
        executable='search_action_node',
        name='search_action_node',
        output='screen',
        parameters=[])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(fly_cmd)
    ld.add_action(search_cmd)

    return ld
