cmake_minimum_required(VERSION 3.8)
project(rear_ackermann_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rear_ackermann_node src/rear_ackermann_node.cpp)
target_include_directories(rear_ackermann_node PRIVATE include)
ament_target_dependencies(rear_ackermann_node ament_index_cpp geometry_msgs rclcpp std_msgs)

install(TARGETS rear_ackermann_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(FILES ../params_setting.json DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_differential_drive test/test_differential_drive.cpp)
  target_include_directories(test_differential_drive PRIVATE include)
endif()

ament_package()
