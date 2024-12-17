### Add gmp_connector Directory as Python Path

```shell
export PYTHONPATH=$PYTHONPATH:$(pwd)
```

# Usage

## Full Pipeline

```shell
python use/full.py
```

Details:

1. Open Gripper
2. Go To Capture Location
3. Capture Image and Save Info
   1. Saves into `storage/captures/[xxx]/data.npy`
   2. Then prints path of saved file
4. Waits for Contact GraspNet processing
   1. Run Contact GraspNet manually
   2. Grasping poses should be saved in `storage/grasping_poses/data.npz`
   3. Press any key to progress
5. Call AGR _(Adaptive Goal Region)_
   1. Saves results into `storage/spline_finals/agr_output.txt`
