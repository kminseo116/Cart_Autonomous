cmake_minimum_required(VERSION 3.8)
project(patient_approach)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/approach_manager.py scripts/depth_patient_point.py
  scripts/patient_depth_batch.py scripts/batch_approach_manager.py
  scripts/lidar_clearance.py
  test/ros_simulation.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md BATCH_RUN.md HARDWARE_BODY_RUN.md DESTINATION share/${PROJECT_NAME})
ament_package()
