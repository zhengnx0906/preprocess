cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_downsampling)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_downsampler src/pointcloud_downsampling.cpp)

add_dependencies(pointcloud_downsampler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pointcloud_downsampler
  ${catkin_LIBRARIES}
)

