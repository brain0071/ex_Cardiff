cmake_minimum_required(VERSION 3.0.2)
project(test_open_manipulator_common_bringup)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES open_manipulator_moveit_demo
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
)


include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


if(gazebo_ros_link_attacher_FOUND)
	add_definitions(-DGAZEBO_LINK_ATTACHER)
endif()

add_executable(open_manipulator_moveit_demo src/moveit_demo.cpp)
target_link_libraries(open_manipulator_moveit_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(open_manipulator_moveit_demo ${catkin_EXPORTED_TARGETS})

catkin_add_env_hooks(99.test_open_manipulator_common_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
