cmake_minimum_required(VERSION 3.5)
project(lidar_processor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# specific pcl stuff
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add executables
add_executable(lidarvis_node src/lidar_visualizer.cpp)
ament_target_dependencies(lidarvis_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(lidarvis_node ${PCL_LIBRARIES})

add_executable(lidarfil_node src/lidar_filterer.cpp)
ament_target_dependencies(lidarfil_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(lidarfil_node ${PCL_LIBRARIES})

add_executable(lidarconcat_node src/lidar_concatenator.cpp)
ament_target_dependencies(lidarconcat_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(lidarconcat_node ${PCL_LIBRARIES})

add_executable(lidarseg_node src/lidar_segmenter.cpp)
ament_target_dependencies(lidarseg_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(lidarseg_node ${PCL_LIBRARIES})

add_executable(lidarstats_node src/lidar_stats.cpp)
ament_target_dependencies(lidarstats_node rclcpp sensor_msgs)

# install targets so ROS2 can find executable
install(TARGETS
  lidarvis_node
  lidarfil_node
  lidarconcat_node
  lidarseg_node
  lidarstats_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
