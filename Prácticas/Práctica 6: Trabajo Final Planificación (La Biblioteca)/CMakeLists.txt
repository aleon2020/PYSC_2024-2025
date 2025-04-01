cmake_minimum_required(VERSION 3.5)
project(trabajo_plansys2_envidio33)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_bt_actions
)

include_directories(include)

add_library(plansys2_envidio33_move_bt_node SHARED src/behavior_tree_nodes/Move.cpp)
list(APPEND plugin_libs plansys2_envidio33_move_bt_node)

add_library(plansys2_envidio33_action_bt_node SHARED src/behavior_tree_nodes/Action.cpp)
list(APPEND plugin_libs plansys2_envidio33_action_bt_node)

add_library(plansys2_envidio33_drop_bt_node SHARED src/behavior_tree_nodes/Drop.cpp)
list(APPEND plugin_libs plansys2_envidio33_drop_bt_node)

add_library(plansys2_envidio33_pick_bt_node SHARED src/behavior_tree_nodes/Pick.cpp)
list(APPEND plugin_libs plansys2_envidio33_pick_bt_node)

add_library(plansys2_envidio33_do_cube_bt_node SHARED src/behavior_tree_nodes/DoCube.cpp)
list(APPEND plugin_libs plansys2_envidio33_do_cube_bt_node)

add_library(plansys2_envidio33_leave_bt_node SHARED src/behavior_tree_nodes/Leave.cpp)
list(APPEND plugin_libs plansys2_envidio33_leave_bt_node)

add_library(plansys2_envidio33_pick_up_bt_node SHARED src/behavior_tree_nodes/PickUp.cpp)
list(APPEND plugin_libs plansys2_envidio33_pick_up_bt_node)

add_library(plansys2_envidio33_say_shhh_bt_node SHARED src/behavior_tree_nodes/SayShhh.cpp)
list(APPEND plugin_libs plansys2_envidio33_say_shhh_bt_node)

add_library(plansys2_envidio33_stand_up_bt_node SHARED src/behavior_tree_nodes/StandUp.cpp)
list(APPEND plugin_libs plansys2_envidio33_stand_up_bt_node)

add_library(plansys2_envidio33_stand_down_bt_node SHARED src/behavior_tree_nodes/StandDown.cpp)
list(APPEND plugin_libs plansys2_envidio33_stand_down_bt_node)

add_library(plansys2_envidio33_search_bt_node SHARED src/behavior_tree_nodes/Search.cpp)
list(APPEND plugin_libs plansys2_envidio33_search_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(trabajo_plansys2_envidio33_controller_node src/trabajo_plansys2_envidio33_controller_node.cpp) 
ament_target_dependencies(trabajo_plansys2_envidio33_controller_node ${dependencies})

add_executable(nav2_sim_node src/nav2_sim_node.cpp)
ament_target_dependencies(nav2_sim_node ${dependencies})

install(DIRECTORY launch pddl behavior_trees_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  trabajo_plansys2_envidio33_controller_node
  nav2_sim_node
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()