cmake_minimum_required(VERSION 3.5)
project(champ_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ompl REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)



add_executable(path_planner src/path_planner.cpp)
ament_target_dependencies(path_planner rclcpp geometry_msgs nav_msgs ompl tf2 tf2_geometry_msgs sensor_msgs visualization_msgs PCL pcl_conversions tf2_sensor_msgs)
target_include_directories(path_planner PUBLIC
  ${OMPL_INCLUDE_DIRS})
target_link_libraries(path_planner
  ${OMPL_LIBRARIES})


add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp geometry_msgs nav_msgs tf2 tf2_ros)

install(TARGETS path_planner controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
