# This CMake file was generated as part of the package creation process.
# This process is documented here:
# https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
#
# This CMake file was modified based on the CMake file found here:
# https://github.com/ros2/demos/blob/galactic/composition/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(uavrt_connection)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_library(connection_component SHARED
  src/connection_component.cpp)
ament_target_dependencies(connection_component
  "rclcpp")

add_executable(main
  src/main.cpp)

target_link_libraries(main
  connection_component)

ament_target_dependencies(main
  "rclcpp")

target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  connection_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

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
