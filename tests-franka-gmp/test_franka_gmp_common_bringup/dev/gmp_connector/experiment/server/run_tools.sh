#!/bin/bash

# SCRIPT PATHS

CAPTURE_PYTHON_SCRIPT="$HOME/dev/gmp_connector/src/capture.py"
TRANSFORMATION_PYTHON_SCRIPT="$HOME/dev/gmp_connector/src/transform_and_cluster.py"
CONDA_PATH=$HOME/anaconda3/etc/profile.d/conda.sh

# Default values
env_name=""
view=false
step=0

# Parse arguments
while getopts "e:vs:" opt; do
  case $opt in
    e) env_name="$OPTARG"
       ;;
    v) view=true
       ;;
    s) step="$OPTARG"
       ;;
    \?) echo "Invalid option -$OPTARG" >&2
        exit 1
       ;;
  esac
done

if [ -z "$env_name" ]; then
    echo "Environment name (-e) is required."
    exit 1
fi

target_dir="$HOME/dev/gmp_connector/storage/$env_name"
export PYTHONPATH=$PYTHONPATH:"$HOME/dev/gmp_connector"
export PYTHONPATH=$PYTHONPATH:"$HOME/dev/UoisSegmentation"

# STEP 003
#
# SEGMENTATION (RETURN FILTERED DEPTH) [~CONDA/UOIS] -> main [seg_data.np]
#

source $CONDA_PATH
conda activate uoisenv
echo $PYTHONPATH
cd $HOME/dev/UoisSegmentation
export PYTHONPATH=$PYTHONPATH:$(pwd)

python $HOME/dev/UoisSegmentation/process.py -e "$env_name" -i "$target_dir/temp"

# STEP 004
#
# CONTACT GRASP-NET [~CONDA/CONTACT_GRASPNET_ENV] -> main [poses.npz]
#

source $CONDA_PATH
conda activate graspnet
cd $HOME/dev/ContactGraspNet/
export PYTHONPATH=$PYTHONPATH:$(pwd)
python contact_graspnet/inference.py --np_path=$target_dir/temp/seg_data.npy --forward_passes=10 --filter_grasps --z_range=[0.6,1.2] --output_dir="$target_dir/temp/"
conda deactivate


