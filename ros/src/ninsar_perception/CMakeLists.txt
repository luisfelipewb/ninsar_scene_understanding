cmake_minimum_required(VERSION 3.16)
project(ninsar_perception)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(segmentation_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common filters search segmentation)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# Pure clustering/box code, shared by the node and the tests (not installed).
add_library(clustering STATIC src/clustering.cpp)
target_include_directories(clustering PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(clustering PUBLIC
  ${PCL_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen
  ${sensor_msgs_TARGETS} ${vision_msgs_TARGETS})

add_executable(object_detector src/object_detector_node.cpp)
target_link_libraries(object_detector
  clustering rclcpp::rclcpp
  ${segmentation_msgs_TARGETS} ${visualization_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})

install(TARGETS object_detector DESTINATION lib/${PROJECT_NAME})

# Python nodes: the ninsar_perception module plus one launcher script per node.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/semantic_cloud_adapter scripts/seg_label_colorizer DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_clustering test/test_clustering.cpp)
  target_link_libraries(test_clustering clustering)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_semantic_cloud_adapter test/test_semantic_cloud_adapter.py)
endif()

ament_package()
