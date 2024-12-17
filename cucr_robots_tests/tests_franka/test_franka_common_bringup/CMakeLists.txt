cmake_minimum_required(VERSION 3.0.2)
project(test_franka_common_bringup)

add_compile_options(-std=c++14)


find_package(gazebo_ros_link_attacher QUIET)
find_package( OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})




if(gazebo_ros_link_attacher_FOUND)
find_package(catkin REQUIRED
  COMPONENTS 
    roscpp
    rospy
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    gazebo_ros_link_attacher
    moveit_msgs
    gazebo_msgs
    cv_bridge
    OpenCV
    image_transport

)
else()
find_package(catkin REQUIRED
  COMPONENTS 
    roscpp
    rospy
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
    gazebo_msgs
    cv_bridge
    OpenCV
    image_transport

)
endif()
find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
    CATKIN_DEPENDS
)


include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
if(gazebo_ros_link_attacher_FOUND)
    add_definitions(-DGAZEBO_LINK_ATTACHER)
endif()

add_executable(franka_move src/franka_move.cpp)
target_link_libraries(franka_move ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(franka_move ${catkin_EXPORTED_TARGETS})


add_executable(franka_pick src/franka_pick.cpp)
target_link_libraries(franka_pick ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(franka_pick ${catkin_EXPORTED_TARGETS})


install(DIRECTORY models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


catkin_add_env_hooks(99.test_franka_common_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
