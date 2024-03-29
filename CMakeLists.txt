# This CMake file was generated as part of the package creation process.
# This process is documented here:
# https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
#
# This CMake file was modified based on the CMake file found here:
# https://github.com/ros2/demos/blob/galactic/composition/CMakeLists.txt
#
# Note: Outside of the use of ament_cmake, there does not appear to be a
# consistent methodology to the commands used with ROS 2 package CMake files.
#
# The methodology followed in this CMake file essentially creates a library for
# each .cpp and then links that .cpp file with the main executable. Since these
# .cpp files do not contain a main, they are more akin to ROS 2 components.
# However, I didn't utilize the "register component" functionality since it
# didn't seem necessary at the time.
# If you would like to know more about components and plugins, I suggest the
# following blog post as a start:
# https://ubuntu.com/blog/components-vs-plugins-in-ros-2
# And then this ROS 2 documentation:
# https://docs.ros.org/en/galactic/Concepts/About-Composition.html
#
# For more info on ament_cmake (ROS 2):
# https://docs.ros.org/en/galactic/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.8)
project(uavrt_connection)

# Default to C++20
# std::lerp functionality requires C++20
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies: uncomment the following section in order to fill in
# further dependencies manually.
# Make sure you add the dependencies to the package.xml file as well!
# Else the program will crash and it won't tell you why.
# find_package(<dependency> REQUIRED)

# ROS 2 required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ROS 2 interfaces
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# In addition to requiring the MAVSDK package and adding it as a dependency,
# you will need to add specifically link the core mavsdk library.
# https://mavsdk.mavlink.io/main/en/cpp/api_changes.html#linking
find_package(MAVSDK REQUIRED)

# Aug 24 2022: Install all Boost 1.71.0 libraries on Ubuntu with:
# sudo apt update
# sudo apt upgrade
# sudo apt install libboost-all-dev
# https://cmake.org/cmake/help/latest/module/FindBoost.html
find_package(Boost)

# UAV-RT packages
find_package(uavrt_interfaces REQUIRED)

include_directories(include)

# ---> Add boost to target dep?

# libtelemetry_component.so
add_library(telemetry_component SHARED
  src/telemetry_component.cpp)
ament_target_dependencies(telemetry_component
  rclcpp
  std_msgs
  geometry_msgs
  MAVSDK
  uavrt_interfaces)

# libcommand_component.so
add_library(command_component SHARED
  src/command_component.cpp)
ament_target_dependencies(command_component
  rclcpp
  MAVSDK
  uavrt_interfaces)

add_executable(main
  src/main.cpp)

target_link_libraries(main
  telemetry_component
  command_component
  MAVSDK::mavsdk)

install(TARGETS
  telemetry_component
  command_component
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
