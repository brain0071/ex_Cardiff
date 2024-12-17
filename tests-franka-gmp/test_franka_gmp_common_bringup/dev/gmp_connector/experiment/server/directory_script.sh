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

# --------------------------

# STEP 001
#
# ARRANGE DIRECTORY FOR ENVIRONMENT
#

# Define the target directory
target_dir="$HOME/dev/gmp_connector/storage/$env_name"
export PYTHONPATH=$PYTHONPATH:"$HOME/dev/gmp_connector"
export PYTHONPATH=$PYTHONPATH:"$HOME/dev/UoisSegmentation"

# Create environment directory if it does not exist
if [ ! -d "$target_dir" ]; then
    mkdir -p "$target_dir"
fi

# Rename "temp" directory and create a new one
if [ -d "$target_dir/temp" ]; then
    # Find the last number in the directory
    last_num=$(ls -v "$target_dir" | grep -E '^[0-9]+$' | tail -n 1)
    next_num=$((last_num + 1))

    # Rename "temp" to the next number
    mv "$target_dir/temp" "$target_dir/$next_num"
fi

# Create a new "temp" directory
mkdir "$target_dir/temp"

