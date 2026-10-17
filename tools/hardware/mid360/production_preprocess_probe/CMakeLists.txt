cmake_minimum_required(VERSION 3.16)
project(mid360_preprocess_probe)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
get_filename_component(AEROMIND_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
add_executable(mid360_preprocess_probe main.cpp)
target_compile_features(mid360_preprocess_probe PRIVATE cxx_std_17)
target_include_directories(mid360_preprocess_probe PRIVATE
  "${AEROMIND_ROOT}/src/third_party/spark-fast-lio/spark_fast_lio/include"
  ${PCL_INCLUDE_DIRS})
ament_target_dependencies(mid360_preprocess_probe rclcpp sensor_msgs pcl_conversions)
target_link_libraries(mid360_preprocess_probe
  "${AEROMIND_ROOT}/install_mid360/spark_fast_lio/lib/libspark_lio_component.so" ${PCL_LIBRARIES})
