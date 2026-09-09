cmake_minimum_required(VERSION 3.8)
project(zlac8015d_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

find_path(MODBUS_INCLUDE_DIR NAMES modbus/modbus.h REQUIRED)
find_library(MODBUS_LIBRARY NAMES modbus REQUIRED)

add_library(zlac8015d_modbus src/modbus_interface.cpp)
target_include_directories(zlac8015d_modbus PUBLIC include ${MODBUS_INCLUDE_DIR})
target_link_libraries(zlac8015d_modbus ${MODBUS_LIBRARY})

add_executable(zlac8015d_driver_node src/zlac8015d_driver_node.cpp)
target_include_directories(zlac8015d_driver_node PRIVATE include)
target_link_libraries(zlac8015d_driver_node zlac8015d_modbus)
ament_target_dependencies(zlac8015d_driver_node rclcpp sensor_msgs std_msgs std_srvs)

add_executable(wheel_odometry_node src/wheel_odometry_node.cpp)
ament_target_dependencies(wheel_odometry_node rclcpp nav_msgs geometry_msgs sensor_msgs tf2_ros)

add_executable(imu_degrees_node src/imu_degrees_node.cpp)
ament_target_dependencies(imu_degrees_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS zlac8015d_modbus zlac8015d_driver_node wheel_odometry_node imu_degrees_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
