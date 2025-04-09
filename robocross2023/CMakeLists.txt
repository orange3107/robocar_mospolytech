cmake_minimum_required(VERSION 3.8)
project(robocross2023)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(nav_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_sensor_msgs  REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(mapping scripts/mapping.cpp)
target_link_libraries(mapping ${PCL_LIBRARIES} yaml-cpp)
ament_target_dependencies(mapping

  "sensor_msgs"
  "rclcpp"
  "pcl_conversions"
  "OpenCV"
  "nav_msgs"
  "visualization_msgs"
  "cv_bridge"
  "ament_index_cpp"
  "yaml-cpp"
  "tf2_ros"
  "tf2_geometry_msgs"
  "tf2_sensor_msgs"
  "tf2_eigen"
  "tf2"
)


add_executable(create_2d_map scripts/create_2d_map.cpp) 
target_link_libraries(create_2d_map ${PCL_LIBRARIES} yaml-cpp)
ament_target_dependencies(create_2d_map

  "sensor_msgs"
  "rclcpp"
  "pcl_conversions"
  "OpenCV"
  "nav_msgs"
  "visualization_msgs"
  "cv_bridge"
  "ament_index_cpp"
  "std_srvs"
  "rclcpp"
  "yaml-cpp"
)

add_executable(pure_pursuit scripts/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit

  "sensor_msgs"
  "rclcpp"
  "pcl_conversions"
  "OpenCV"
  "nav_msgs"
  "visualization_msgs"
  "cv_bridge"
  "geometry_msgs"
)

install(TARGETS
  mapping
  pure_pursuit
  create_2d_map
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  PROGRAMS
  scripts/mission_control.py
  scripts/serial_odom.py
  scripts/pure_pursuit.cpp
  scripts/get_param_from_ui.py
  scripts/go_global_path.py
  scripts/hybrid_a_star.py

  scripts/create_2d_map.cpp
  scripts/mapping.cpp
  scripts/create_path.py
  scripts/car_in_global_map.py
  scripts/transforms.py
  scripts/teleop.py
  scripts/test_r_auto.py
  scripts/calc_odom.py

  scripts/calc_odom_sim.py
  scripts/pcd_to_map.py
  scripts/calc_odom.py
  scripts/ser_move.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    scripts
    launch
    paths
    maps
    worlds
    models
    rviz
    params
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
