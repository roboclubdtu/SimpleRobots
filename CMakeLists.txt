cmake_minimum_required(VERSION 3.8)
project(simple_robots NONE)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include("cmake/gen.cmake")

generate_robot(DIR "scara")

install(
    DIRECTORY rviz2
    DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()