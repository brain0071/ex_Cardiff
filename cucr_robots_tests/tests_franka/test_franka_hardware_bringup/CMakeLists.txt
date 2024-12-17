cmake_minimum_required(VERSION 3.0.2)
project(test_franka_hardware_bringup)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    gazebo_ros_link_attacher
    moveit_msgs
    gazebo_msgs
)


catkin_package(
)




include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


add_executable(franka_real_demo scripts/franka_real_demo.cpp)
target_link_libraries(franka_real_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(franka_real_demo ${catkin_EXPORTED_TARGETS})

catkin_add_env_hooks(99.test_franka_hardware_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

