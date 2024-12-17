cmake_minimum_required(VERSION 3.0.2)
project(test_stretch2_simulation_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_add_env_hooks(99.test_stretch2_simulation_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
