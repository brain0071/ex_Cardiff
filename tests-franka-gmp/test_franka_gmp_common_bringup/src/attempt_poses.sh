#!/bin/bash

# STEP 001
#
# RUN FRANKA SIMULATION
# gnome-terminal --tab --title="gazebo" --command="bash -c 'roslaunch test_franka_gmp_simulation_bringup full.launch'" && wait


# STEP 002
#
# MOVE TO CAPTURE POSITION
rosrun test_franka_gmp_common_bringup move_to_capture.py && wait
sleep 7

# STEP 003
#
# ATTEMPT EACH GRASP POSE UNTIL SUCCESSFULL
rosrun test_franka_gmp_common_bringup move_to_poses.py && wait


# STEP 004
#
# CLOSE TERMINALS
gnome-terminal -- bash -c "pkill gnome-terminal"
sleep 10;