cmake_minimum_required(VERSION 3.5)
project(exploration_example)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    plansys2_msgs
    plansys2_executor
)

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(collect_action_node src/collect_action_node.cpp)
ament_target_dependencies(collect_action_node ${dependencies})

add_executable(drop_action_node src/drop_action_node.cpp)
ament_target_dependencies(drop_action_node ${dependencies})

add_executable(analyse_soil_action_node src/analyse_soil_action_node.cpp)
ament_target_dependencies(analyse_soil_action_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  collect_action_node
  drop_action_node
  analyse_soil_action_node
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