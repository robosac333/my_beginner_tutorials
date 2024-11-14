cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${rosidl_default_runtime_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Add the executable
add_executable(talker
  src/publisher_member_function.cpp

)
add_executable(listener
  src/subscriber_member_function.cpp
)

# Add dependencies
ament_target_dependencies(talker
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_ros
  geometry_msgs
)

ament_target_dependencies(listener
  rclcpp
  std_msgs
)

# Set include directories for the targets
target_include_directories(talker PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

target_include_directories(listener PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

# Ensure the generated headers are available
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the executable
install(TARGETS talker listener
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
