cmake_minimum_required(VERSION 3.5)
project(groundgrid)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_loader REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_rviz_plugin REQUIRED)
find_package(grid_map_visualization REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  src/GroundGrid.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
  grid_map_core grid_map_ros grid_map_cv grid_map_loader
  grid_map_msgs grid_map_rviz_plugin grid_map_visualization
  cv_bridge image_transport sensor_msgs pcl_ros pcl_conversions
  OpenCV PCL Eigen3
)

add_library(${PROJECT_NAME}_groundsegmentation_lib
  src/GroundSegmentation.cpp
)
ament_target_dependencies(${PROJECT_NAME}_groundsegmentation_lib
  rclcpp geometry_msgs sensor_msgs grid_map_core grid_map_ros grid_map_cv
  pcl_ros pcl_conversions OpenCV PCL Eigen3
)

add_executable(${PROJECT_NAME}_node
  src/groundgrid_ros2_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_groundsegmentation_lib
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_groundsegmentation_lib
)

install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_groundsegmentation_lib
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
