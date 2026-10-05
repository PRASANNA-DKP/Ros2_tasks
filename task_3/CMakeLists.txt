cmake_minimum_required(VERSION 3.8)
project(task_3)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToCharger.srv"
  "action/GoToCharger.action"
)

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server
  rclcpp
)

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client
  rclcpp
)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client
  rclcpp
  rclcpp_action
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(service_server "${cpp_typesupport_target}")
target_link_libraries(service_client "${cpp_typesupport_target}")
target_link_libraries(action_server "${cpp_typesupport_target}")
target_link_libraries(action_client "${cpp_typesupport_target}")

install(TARGETS
  service_server
  service_client
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()