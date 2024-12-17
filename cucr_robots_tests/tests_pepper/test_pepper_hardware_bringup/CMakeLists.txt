cmake_minimum_required(VERSION 3.0.2)
project(test_pepper_hardware_bringup)

find_package(catkin REQUIRED COMPONENTS
    rospy
)


catkin_package(
)
include_directories(
)


catkin_add_env_hooks(99.test_pepper_hardware_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

