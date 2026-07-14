cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(crawler_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)

set(DEPENDENCIES
  ament_index_cpp
  gazebo_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_library(crawler_gazebo_drive SHARED
  src/crawler_gazebo_drive_plugin.cpp
)
target_include_directories(crawler_gazebo_drive PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
ament_target_dependencies(crawler_gazebo_drive
  geometry_msgs
  rclcpp
)
target_link_libraries(crawler_gazebo_drive
  ${GAZEBO_LIBRARIES}
)
install(TARGETS crawler_gazebo_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(crawler_ignition_control_bridge
  src/crawler_ignition_control_bridge.cpp
)
ament_target_dependencies(crawler_ignition_control_bridge
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)
target_link_libraries(crawler_ignition_control_bridge
  ignition-msgs8::ignition-msgs8
  ignition-transport11::ignition-transport11
)
install(TARGETS crawler_ignition_control_bridge
  DESTINATION lib/${PROJECT_NAME})

function(add_crawler_gazebo_node target source)
  add_executable(${target} ${source})
  target_include_directories(${target} PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(${target} ${PCL_LIBRARIES})
  ament_target_dependencies(${target} ${DEPENDENCIES})
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

add_crawler_gazebo_node(pc2_frame_change src/pc2_frame_change.cpp)
add_crawler_gazebo_node(pc2_frame_change_se2 src/pc2_frame_change_se2.cpp)
add_crawler_gazebo_node(nearest_xy_height_goal_node src/nearest_xy_height_goal_node.cpp)
add_crawler_gazebo_node(gazebo_to_octomap_publisher src/point_cloud/gazebo_to_octomap_publisher.cpp)
add_crawler_gazebo_node(cloudmap_to_pcd src/point_cloud/cloudmap_to_pcd.cpp)
add_crawler_gazebo_node(voxel_overhang_removal src/point_cloud/voxel_overhang_removal.cpp)

install(PROGRAMS
    src/spawn_robocup_arena.py
    scripts/generate_crawler_urdf.py
    scripts/ramdom_world.py
  DESTINATION lib/${PROJECT_NAME}
  OPTIONAL)

install(DIRECTORY config gazebo_model launch pcd stl urdf world
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  OPTIONAL)

ament_package()
