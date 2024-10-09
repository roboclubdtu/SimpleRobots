cmake_minimum_required(VERSION 3.8)
project(simple_robots)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include("cmake/gen.cmake")

message("Configuring ${PROJECT_NAME} project")

# Generating common library
message("Generating common library")
file(GLOB_RECURSE lib_src LIST_DIRECTORIES false "${CMAKE_CURRENT_LIST_DIR}/src/**")
ament_auto_add_library(${PROJECT_NAME} ${lib_src})

# Making robot resources
message("Generating robots ROS2 resources")
generate_robot(DIR "scara")

# Installing RViz2 resources
install(
    DIRECTORY rviz2
    DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
message("--------- Configuration done ---------")