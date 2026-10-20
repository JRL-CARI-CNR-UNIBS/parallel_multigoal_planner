cmake_minimum_required(VERSION 3.16)
project(parallel_multigoal_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(parallel_multigoal_planner_msgs REQUIRED)
find_package(ik_solver_msgs REQUIRED)
find_package(graph_core REQUIRED)
find_package(mirrt_star REQUIRED)
find_package(cnr_logger REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/service_collision_checker.cpp
  src/${PROJECT_NAME}/goal_set_builder.cpp
  src/${PROJECT_NAME}/parallel_multigoal_planner_action_server.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  graph_core::graph_core
  mirrt_star::mirrt_star
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rclcpp_action
  moveit_msgs
  trajectory_msgs
  ament_index_cpp
  parallel_multigoal_planner_msgs
  ik_solver_msgs
  cnr_logger
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_node PUBLIC
  ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
  rclcpp
  rclcpp_action
  ament_index_cpp
  moveit_msgs
  parallel_multigoal_planner_msgs
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_display_planned_path.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  moveit_msgs
  trajectory_msgs
  parallel_multigoal_planner_msgs
  ik_solver_msgs
  graph_core
  mirrt_star
  cnr_logger
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
