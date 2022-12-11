cmake_minimum_required(VERSION 3.5)
project(package1)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)




add_executable(node1 src/node1.cpp)
ament_target_dependencies(node1 rclcpp std_msgs)

add_executable(node2 src/node2.cpp)
ament_target_dependencies(node2 rclcpp std_msgs)


install(TARGETS
  node1
  node2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
