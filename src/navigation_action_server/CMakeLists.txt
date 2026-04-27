cmake_minimum_required(VERSION 3.8)
project(navigation_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(turtlesim REQUIRED)

add_library(nav_action_server_lib SHARED
  src/nav_action_server.cpp
)
target_include_directories(nav_action_server_lib PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(nav_action_server_lib PRIVATE "NAVIGATION_ACTION_SERVER_BUILDING_DLL")
ament_target_dependencies(nav_action_server_lib
  rclcpp
  rclcpp_components
  rclcpp_action
  tf2
  tf2_ros
  geometry_msgs
  navigation_interfaces
  turtlesim
)
rclcpp_components_register_node(nav_action_server_lib PLUGIN "nav_action_server_lib::NavActionServer" EXECUTABLE nav_action_server_node)

add_library(nav_action_client_lib SHARED
  src/nav_action_client.cpp
)
target_include_directories(nav_action_client_lib PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(nav_action_client_lib PRIVATE "NAVIGATION_ACTION_SERVER_BUILDING_DLL")
ament_target_dependencies(nav_action_client_lib
  rclcpp
  rclcpp_components
  rclcpp_action
  geometry_msgs
  navigation_interfaces
)
rclcpp_components_register_node(nav_action_client_lib PLUGIN "nav_action_client_lib::NavActionClient" EXECUTABLE nav_action_client_node)

install(TARGETS
  nav_action_server_lib
  nav_action_client_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
