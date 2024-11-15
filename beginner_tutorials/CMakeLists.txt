cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Create compilation database
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Enable testing
if(BUILD_TESTING)
  enable_testing()
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
endif()

# Include directories
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

# Create publisher library without main function
add_library(publisher_lib
  src/publisher_member_function.cpp
)

# Remove main function from library compilation
target_compile_definitions(publisher_lib PRIVATE BUILDING_LIBRARY)

# Add dependencies to the library
ament_target_dependencies(publisher_lib
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_ros
  geometry_msgs
)

# Create the integration test executable
add_executable(integration_test_node
  src/integration_test_node.cpp
)

# Link the test with the publisher library
target_link_libraries(integration_test_node
  publisher_lib
  catch_ros2::catch_ros2_with_node_main
)

ament_target_dependencies(integration_test_node
  rclcpp
  std_srvs
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
)

# Create the talker executable with main function
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

# Install targets
install(TARGETS
  publisher_lib
  integration_test_node
  talker
  listener
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install includes
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Add integration test support with a meaningful name
  catch_ros2_add_integration_test(publisher_message_test
    LAUNCH_FILE integration_test.launch.yaml
  )
endif()

ament_package()
