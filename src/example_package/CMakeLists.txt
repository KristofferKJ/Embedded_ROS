cmake_minimum_required(VERSION 3.5)
project(example_package)

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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Include header files from the include directory
include_directories(
  include
)

add_executable(image_subscriber
	src/image_subscriber.cpp
)
add_executable(image_downscaler
  src/image_downscaler.cpp
)
add_executable(bram_node
  src/bram_node.cpp
)
add_executable(keyboard_node
  src/keyboard_node.cpp
)

ament_target_dependencies(image_subscriber
	rclcpp
	sensor_msgs
	OpenCV
	cv_bridge
)
ament_target_dependencies(image_downscaler
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
)
ament_target_dependencies(bram_node
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
  std_msgs
)
ament_target_dependencies(keyboard_node
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
  std_msgs
)

install(TARGETS image_subscriber DESTINATION lib/${PROJECT_NAME})
install(TARGETS image_downscaler DESTINATION lib/${PROJECT_NAME})
install(TARGETS bram_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS keyboard_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
