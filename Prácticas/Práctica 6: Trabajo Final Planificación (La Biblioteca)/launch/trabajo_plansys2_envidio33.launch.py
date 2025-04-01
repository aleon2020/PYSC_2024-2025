import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('trabajo_plansys2_envidio33')
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
          'model_file': example_dir + '/pddl/robot_domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions

    # Move
    move_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1672,
            'server_port': 1673,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])

    # Pick
    pick_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'pick',
            'publisher_port': 1674,
            'server_port': 1675,
            'bt_xml_file': example_dir + '/behavior_trees_xml/pick.xml'
          }
        ])
    
    # Drop
    drop_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='drop',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'drop',
            'publisher_port': 1676,
            'server_port': 1677,
            'bt_xml_file': example_dir + '/behavior_trees_xml/drop.xml'
          }
        ])
    
    # Solve
    solve_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='solve',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'solve',
            'publisher_port': 1678,
            'server_port': 1679,
            'bt_xml_file': example_dir + '/behavior_trees_xml/solve.xml'
          }
        ])

    # Guide Visitor
    guide_visitor_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='guide_visitor',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'guide_visitor',
            'publisher_port': 1680,
            'server_port': 1681,
            'bt_xml_file': example_dir + '/behavior_trees_xml/guide_visitor.xml'
          }
        ])
    
    # Search Book
    search_book_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='search_book',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'search_book',
            'publisher_port': 1682,
            'server_port': 1683,
            'bt_xml_file': example_dir + '/behavior_trees_xml/search_book.xml'
          }
        ])
    
    # Move Object
    move_object_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_object',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move_object',
            'publisher_port': 1684,
            'server_port': 1685,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move_object.xml'
          }
        ])
    
    # Shut Up
    shut_up_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='shut_up',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'shut_up',
            'publisher_port': 1686,
            'server_port': 1687,
            'bt_xml_file': example_dir + '/behavior_trees_xml/shut_up.xml'
          }
        ])

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_cmd)
    ld.add_action(pick_cmd)
    ld.add_action(drop_cmd)
    ld.add_action(solve_cmd)
    ld.add_action(guide_visitor_cmd)
    ld.add_action(search_book_cmd)
    ld.add_action(move_object_cmd)
    ld.add_action(shut_up_cmd)

    return ld