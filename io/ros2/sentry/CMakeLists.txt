cmake_minimum_required(VERSION 3.16)
project(sentry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp_components REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Send.msg"
  "msg/Receive.msg"
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/node.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  geometry_msgs
)

rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "sentry::Sentry"
  EXECUTABLE ${PROJECT_NAME}_exec
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
