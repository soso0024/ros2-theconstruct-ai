cmake_minimum_required(VERSION 3.8)
project(reworked_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(reworked_services REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(service_client_node src/service_client.cpp)
ament_target_dependencies(service_client_node rclcpp std_srvs)

add_executable(service_client_v2_node src/service_client_v2.cpp)
ament_target_dependencies(service_client_v2_node rclcpp std_srvs)

add_executable(ex42_node src/ex42.cpp)
ament_target_dependencies(ex42_node rclcpp std_srvs geometry_msgs)

add_executable(move_serv_node src/mov_server.cpp)
ament_target_dependencies(move_serv_node rclcpp geometry_msgs reworked_services)

add_executable(move_client_node src/mov_client.cpp)
ament_target_dependencies(move_client_node rclcpp reworked_services)

add_executable(spin_node src/spin.cpp)
ament_target_dependencies(spin_node rclcpp geometry_msgs reworked_services)

add_executable(spin_cli_node src/spin_client.cpp)
ament_target_dependencies(spin_cli_node rclcpp geometry_msgs reworked_services)

install(TARGETS
   service_client_node
   service_client_v2_node
   ex42_node
   move_serv_node
   move_client_node
   spin_node
   spin_cli_node
   DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MyCustomServiceMessage.srv"
  "srv/Spin.srv"
)

ament_package()
