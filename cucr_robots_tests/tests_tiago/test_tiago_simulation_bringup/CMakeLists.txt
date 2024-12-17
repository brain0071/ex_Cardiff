cmake_minimum_required(VERSION 3.0.2)
project(test_tiago_simulation_bringup)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_tiago_simulation_bringup
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

catkin_add_env_hooks(99.test_tiago_simulation_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
