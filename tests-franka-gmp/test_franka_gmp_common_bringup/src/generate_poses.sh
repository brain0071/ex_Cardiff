#!/bin/bash

# SCRIPT PATHS
TRANSFORMATION_PYTHON_SCRIPT="$HOME/ros/noetic/repos/github.com/CardiffUniversityComputationalRobotics/tests-franka-gmp/test_franka_gmp_common_bringup/dev/gmp_connector/src/transform_and_cluster.py"
CONDA_PATH=$HOME/anaconda3/etc/profile.d/conda.sh
target_dir="$HOME/dev/storage"

STEP 001

# RUN FRANKA SIMULATION
gnome-terminal --tab --title="gazebo" --command="bash -c 'roslaunch test_franka_gmp_simulation_bringup full.launch'"
sleep 10


# STEP 002
#
# MOVE TO AND CAPTURE SCENE
gnome-terminal --tab --title="capture" --command="bash -c 'rosrun test_franka_gmp_common_bringup move_to_capture.py'" && wait
rosrun test_franka_gmp_common_bringup gazebo_spawner.py && wait
sleep 1
rosrun test_franka_gmp_common_bringup capture_scene.py && wait
sleep 7

# STEP 003
#
# CONTACT GRASP-NET 
source $CONDA_PATH
conda activate contact_graspnet
cd $HOME/dev/ContactGraspNet
python3 contact_graspnet/inference.py --np_path=$target_dir/scene.npy --forward_passes=10 --z_range=[0.3,1.4] && wait
conda deactivate
cd $HOME



# Function to check if the value meets the condition
check_value_condition() {
    # Extract the third value from the output file or command
    third_value=$(python3 -c "import sys; data = open(sys.argv[1]).read().strip().split(); print(float(data[2]))" "$target_dir/agr_output_single.txt")
    
    # Compare the third value with 0.5
    if (( $(echo "$third_value > 0.5" | bc -l) )); then
        return 0  # Return success if condition is met
    else
        return 1  # Return failure if condition is not met
    fi
}

# Initial run of transformation and clustering
echo "Running transformation and clustering..."
python3 "$TRANSFORMATION_PYTHON_SCRIPT" -i "$HOME/dev/ContactGraspNet/results/predictions_scene.npz" -o "$OUTPUT_DIR" && wait

# Check the condition after the initial run
while ! check_value_condition; do
    echo "Condition not met, running transformation and clustering again..."
    python3 "$TRANSFORMATION_PYTHON_SCRIPT" -i "$HOME/dev/ContactGraspNet/results/predictions_scene.npz" -o "$OUTPUT_DIR" && wait
done

echo "Desired condition (third value > 0.5) is met."
# STEP 005
#
# Generate Collisions for Robowflex

python3 $HOME/ros/noetic/repos/github.com/CardiffUniversityComputationalRobotics/tests-franka-gmp/test_franka_gmp_common_bringup/src/gazebo_listener.py && wait
sleep 1


# STEP 005
#
# CLOSE TERMINALS
# gnome-terminal -- bash -c "pkill gnome-terminal"
