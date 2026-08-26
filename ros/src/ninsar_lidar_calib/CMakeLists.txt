cmake_minimum_required(VERSION 3.16)
project(ninsar_lidar_calib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common filters segmentation)
find_package(Eigen3 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# ---- detection library, shared by the live node and the offline runner ------
add_library(calib_core SHARED
  src/params.cpp
  src/ground_plane.cpp
  src/corner_detector.cpp
  src/detection_log.cpp
  src/markers.cpp
)
target_include_directories(calib_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(calib_core PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(calib_core PUBLIC
  rclcpp sensor_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_eigen pcl_conversions
)

# ---- live node --------------------------------------------------------------
add_executable(calib_node src/calib_node.cpp)
target_link_libraries(calib_node calib_core)
ament_target_dependencies(calib_node rclcpp std_srvs tf2_ros)

# ---- offline runner ---------------------------------------------------------
add_executable(detect_from_bag src/detect_from_bag.cpp)
target_link_libraries(detect_from_bag calib_core)
ament_target_dependencies(detect_from_bag
  rclcpp rosbag2_cpp rosbag2_storage tf2 tf2_ros tf2_msgs
)

install(TARGETS calib_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS calib_node detect_from_bag DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

# ---- python solver ----------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/solve.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME solve
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_detection
    test/test_ground_plane.cpp
    test/test_corner_detector.cpp
  )
  if(TARGET test_detection)
    target_link_libraries(test_detection calib_core)
  endif()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_solve test/test_solve.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(calib_core)
ament_package()
