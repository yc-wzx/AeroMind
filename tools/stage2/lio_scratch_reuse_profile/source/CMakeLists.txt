cmake_minimum_required(VERSION 3.20)
project(spark_fast_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -pthread")
set(CMAKE_CXX_FLAGS "-O3")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++0x -fexceptions")

message(STATUS "Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message(STATUS "core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message(STATUS "core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

set(PCL_FIND_QUIETLY ON)
set(CLEAR_WARNING_SUPPRESSION OFF)
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CLEAR_WARNING_SUPPRESSION ON)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

if (CLEAR_WARNING_SUPPRESSION)
  unset(CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
endif()

include_directories(
  include
  third_party/ikd-Tree
  third_party/IKFoM_toolkit)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#### for ikd-Tree
add_library(ikd_tree third_party/ikd-Tree/ikd_Tree.cpp)
# To resolve "recompile with -fPIC" issue
set_target_properties(ikd_tree PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Optional: Livox driver
find_package(livox_ros_driver QUIET)

# Create the library as a component
add_library(spark_lio_component SHARED
  src/spark_fast_lio.cpp
  src/preprocess.cpp
)

# Example: C++ standard
target_compile_features(spark_lio_component PUBLIC cxx_std_20)

# Link external libs. Replace `ikd_tree` with your actual library name
target_link_libraries(spark_lio_component
  ${PCL_LIBRARIES}
  ikd_tree
)

# If Livox found, link it here
if(livox_ros_driver_FOUND)
  target_link_libraries(spark_lio_component
    ${livox_ros_driver_LIBRARIES}
  )
  target_include_directories(spark_lio_component PRIVATE
    ${livox_ros_driver_INCLUDE_DIRS}
  )
  target_compile_definitions(spark_lio_component PRIVATE
    -DLIVOX_ROS_DRIVER_FOUND=${livox_ros_driver_FOUND}
  )
else()
  message(STATUS "Missing livox driver! AVIA Lidar will be disabled!")
endif()

# Pull in ament dependencies
ament_target_dependencies(
  spark_lio_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)

rclcpp_components_register_node(
  spark_lio_component
  PLUGIN "spark_fast_lio::SPARKFastLIO2"
  EXECUTABLE spark_lio_mapping
)

# Install
install(
  TARGETS spark_lio_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Example for installing launch files, etc.
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Generate package info
ament_package()


# The read-only fixture bridge is retained to match the qualified source snapshot.
target_compile_definitions(spark_lio_component PRIVATE AEROMIND_INPUT_FIXTURE_BRIDGE)
option(AEROMIND_BUILD_ISOLATED_TESTS "Build isolated production tests/replay tool" OFF)
if(AEROMIND_BUILD_ISOLATED_TESTS)
# Isolated test of actual callback/reset methods, never installed.
add_executable(production_prediction_tests ../production_prediction_tests.cpp)
target_compile_options(production_prediction_tests PRIVATE -O0 -g0 -fvisibility=hidden)
target_compile_features(production_prediction_tests PUBLIC cxx_std_20)
target_link_libraries(production_prediction_tests spark_lio_component ${PCL_LIBRARIES} ikd_tree)
ament_target_dependencies(production_prediction_tests rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_eigen pcl_conversions)

# AEROMIND_INPUT_BEGIN_CMAKE

# Isolated direct tests of actual callbacks/sync/init, never installed.
add_executable(production_input_tests ../production_input_tests.cpp)
target_compile_options(production_input_tests PRIVATE -O0 -g0 -fvisibility=hidden)
target_compile_features(production_input_tests PUBLIC cxx_std_20)
target_link_libraries(production_input_tests spark_lio_component ${PCL_LIBRARIES} ikd_tree)
ament_target_dependencies(production_input_tests rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_eigen pcl_conversions)
add_executable(cache_initialization_probe ../cache_initialization_probe.cpp)
target_compile_options(cache_initialization_probe PRIVATE -O0 -g0 -fvisibility=hidden)
target_compile_features(cache_initialization_probe PUBLIC cxx_std_20)
target_link_libraries(cache_initialization_probe spark_lio_component ${PCL_LIBRARIES} ikd_tree)
ament_target_dependencies(cache_initialization_probe rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_eigen pcl_conversions)
add_executable(startup_anchor_tests ../startup_anchor_tests.cpp)
target_compile_options(startup_anchor_tests PRIVATE -O0 -g0 -fvisibility=hidden)
target_compile_features(startup_anchor_tests PUBLIC cxx_std_20)
target_link_libraries(startup_anchor_tests spark_lio_component ${PCL_LIBRARIES} ikd_tree)
ament_target_dependencies(startup_anchor_tests rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_eigen pcl_conversions)
# AEROMIND_INPUT_END_CMAKE

add_executable(callback_replay_probe "${CMAKE_CURRENT_SOURCE_DIR}/../callback_replay_probe.cpp")
target_link_libraries(callback_replay_probe spark_lio_component)
ament_target_dependencies(callback_replay_probe rclcpp sensor_msgs nav_msgs)

add_executable(calibration_tests ../calibration_tests.cpp)
target_compile_options(calibration_tests PRIVATE -O0 -g0 -fvisibility=hidden)
target_link_libraries(calibration_tests spark_lio_component ${PCL_LIBRARIES} ikd_tree)
ament_target_dependencies(calibration_tests rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_eigen pcl_conversions)

add_executable(keyframe_tests ../keyframe_tests.cpp)
target_compile_options(keyframe_tests PRIVATE -O0 -g0 -fvisibility=hidden)
target_link_libraries(keyframe_tests spark_lio_component ${PCL_LIBRARIES} ikd_tree)
ament_target_dependencies(keyframe_tests rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_eigen pcl_conversions)

endif()
