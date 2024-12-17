#!/bin/bash

# Initialize variables
max_iterations=300
target_dir="$HOME/dev/storage"
counter=1
TRANSFORMATION_PYTHON_SCRIPT="$HOME/ros/noetic/repos/github.com/CardiffUniversityComputationalRobotics/tests-franka-gmp/test_franka_gmp_common_bringup/dev/gmp_connector/src/transform_and_cluster.py"
CONDA_PATH=$HOME/anaconda3/etc/profile.d/conda.sh

# Function to create numbered directory
create_numbered_directory() {
    mkdir -p "$target_dir/$counter"
    OUTPUT_DIR="$target_dir/$counter"
}


# Function to run gazebo listener and save results
run_gazebo_listener_and_save() {
    python3 $HOME/ros/noetic/repos/github.com/CardiffUniversityComputationalRobotics/tests-franka-gmp/test_franka_gmp_common_bringup/src/gazebo_listener.py > "$OUTPUT_DIR/"
}

# Function to check if the value meets the condition
check_value_condition() {
    third_value=$(python3 -c "import sys; data = open(sys.argv[1]).read().strip().split(); print(float(data[2]))" "$OUTPUT_DIR/agr_output_single.txt")
    if (( $(echo "$third_value > 0.5" | bc -l) )); then
        return 0
    else
        return 1
    fi
}

# STEP 001
gnome-terminal --tab --title="gazebo" --command="bash -c 'roslaunch test_franka_gmp_simulation_bringup full.launch'"
sleep 10

# Main loop for 100 iterations
for (( counter=1; counter<=$max_iterations; counter++ )); do
    # Create numbered directory for current run
    create_numbered_directory $counter

    # STEP 002
    gnome-terminal --tab --title="capture" --command="bash -c 'rosrun test_franka_gmp_common_bringup move_to_capture.py'" && wait
    rosrun test_franka_gmp_common_bringup gazebo_spawner.py && wait
    rosrun test_franka_gmp_common_bringup capture_scene.py -o "$OUTPUT_DIR" && wait
    sleep 10

    # STEP 003
    source $HOME/anaconda3/etc/profile.d/conda.sh
    conda activate contact_graspnet
    cd $HOME/dev/ContactGraspNet
    python3 contact_graspnet/inference.py --np_path=$OUTPUT_DIR/scene.npy --forward_passes=10 --z_range=[0.3,1.4] && wait
    conda deactivate
    cd $HOME

    # STEP 004
    echo "Running transformation and clustering..."
    python3 "$TRANSFORMATION_PYTHON_SCRIPT" -i "$HOME/dev/ContactGraspNet/results/predictions_scene.npz" -o "$OUTPUT_DIR" && wait
    # Function to check if the value meets the condition
    check_value_condition() {
    third_value=$(python3 -c "import sys; data = open(sys.argv[1]).read().strip().split(); print(float(data[2]))" "$OUTPUT_DIR/agr_output_single.txt")
    if (( $(echo "$third_value > 0.5" | bc -l) )); then
        return 0
    else
        return 1
    fi
}
    # Check the condition after the initial run
    while ! check_value_condition; do
        echo "Condition not met, running transformation and clustering again..."
        python3 "$TRANSFORMATION_PYTHON_SCRIPT" -i "$HOME/dev/ContactGraspNet/results/predictions_scene.npz" -o "$OUTPUT_DIR" && wait
    done

    echo "Desired condition (third value > 0.5) is met."

    # STEP 005
    # gnome-terminal --tab --title="gazebo listener" --command="bash -c 'python3 $HOME/ros/noetic/repos/github.com/CardiffUniversityComputationalRobotics/tests-franka-gmp/test_franka_gmp_common_bringup/src/gazebo_listener.py > "$OUTPUT_DIR"'" && wait
    
    rosrun test_franka_gmp_common_bringup gazebo_listener.py -o "$OUTPUT_DIR" && wait
    rosrun test_franka_gmp_common_bringup gazebo_cleaner.py && wait

    
    # sleep 5

    # gnome-terminal -- bash -c "pkill gnome-terminal"
    # sleep 10
done

# Optional: Close all terminals at the end
# gnome-terminal -- bash -c "pkill gnome-terminal"
