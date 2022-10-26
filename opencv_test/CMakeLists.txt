cmake_minimum_required(VERSION 3.8)
project(opencv_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_executable(opencv_test_1 src/test1.cpp)
add_executable(opencv_test_2 src/test2.cpp)
add_executable(canny_edge src/canny_edge_detector.cpp)
add_executable(harris_corner src/harris_corner.cpp)
add_executable(hough_circles src/hough_circles.cpp)
add_executable(visual_servoing src/vis_srv.cpp)
ament_target_dependencies(opencv_test_1 rclcpp std_msgs sensor_msgs image_transport cv_bridge)
ament_target_dependencies(opencv_test_2 rclcpp std_msgs sensor_msgs image_transport cv_bridge)
ament_target_dependencies(canny_edge rclcpp std_msgs sensor_msgs image_transport cv_bridge)
ament_target_dependencies(harris_corner rclcpp std_msgs sensor_msgs image_transport cv_bridge)
ament_target_dependencies(hough_circles rclcpp std_msgs sensor_msgs image_transport cv_bridge)
ament_target_dependencies(visual_servoing rclcpp std_msgs sensor_msgs controller_manager_msgs image_transport cv_bridge)

install(TARGETS
  opencv_test_1
  opencv_test_2
  canny_edge
  harris_corner
  hough_circles
  visual_servoing
  DESTINATION lib/${PROJECT_NAME})

ament_package()
