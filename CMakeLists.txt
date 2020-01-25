cmake_minimum_required(VERSION 2.8.3)
project(vdm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_ros_planning_interface
)

include_directories( ${catkin_INCLUDE_DIRS})

catkin_package(
)

add_executable(main 
    src/main.cpp
    src/RobotModelTools.cpp
    src/RobotMarkerGenerator.cpp
)

target_link_libraries(main ${catkin_LIBRARIES})