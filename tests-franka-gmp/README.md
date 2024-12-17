# CUCR Lab Demos (Franka GMP). How to Use:

These packages can be installed through [tue-env](https://github.com/CardiffUniversityComputationalRobotics/tue-env).

## Simulation tests

### 0. Installing (checking out) Franka simulation tests

Before installing any new package, update your installed packages:

```bash
cucr-get update
```

To install (check out) the Franka tests in simulation:

```bash
cucr-get install ros-test_franka_gmp_simulation_bringup
```

To build the new packages in either case:

```bash
cucr-make
```

Source updated setup files:

```bash
source ~/.bashrc
```

### 1. Installing Conda Environment for Contact Graspnet

```bash
cd
wget https://repo.anaconda.com/archive/Anaconda3-2021.11-Linux-x86_64.sh
bash ~/Anaconda3-2021.11-Linux-x86_64.sh

source ~/anaconda3/etc/profile.d/conda.sh
conda init



conda config --set auto_activate_base false

conda create --name contact_graspnet python=3.8
conda activate contact_graspnet
conda install -c conda-forge cudatoolkit=11.3
conda install pytorch torchvision
conda install -c conda-forge cudnn=8.2
pip install tensorflow==2.5
pip install tensorflow-gpu==2.5
pip install opencv-python-headless
pip install pyyaml==5.4.1
pip install pyrender
pip install tqdm
pip install numpy==1.20.3
pip install mayavi
pip install PyQt5
pip install trimesh==4.0.4
sudo apt install nvidia-cuda-toolkit
conda install -c conda-forge configobj

```

```bash
cd ~/dev/ContactGraspNet
conda activate contact_graspnet
sh compile_pointnet_tfops.sh
```

:warning: **Warning** :warning: Contact GraspNet must be installed as a prerequisite. To download trained model please check readme file for ContactGraspNet

These model can be downloaded through
[ContactGraspNet](https://drive.google.com/drive/folders/1tBHKf60K8DLM5arm-Chyf7jxkzOr5zGl) and copy them into the `checkpoints/` folder.

<!-- [UoisSegmentation](https://drive.usercontent.google.com/download?id=1D-eaiOgFq_mg8OwbLXorgOB5lrxvmgQd&export=download&authuser=0) -->

### 2. Grasp Pose Generation in Gazebo:

```bash
roscd test_franka_gmp_common_bringup/src
```

```bash
chmod +x generate_poses.sh attempt_poses.sh capture_scene.py move_to_capture.py move_to_poses.py gazebo_listener.py gazebo_spawner.py
```

Run the bash script:

```bash
rosrun test_franka_gmp_common_bringup generate_poses.sh
```

### 3. Attempting Grasp Poses in Gazebo:

Run the bash script:

```bash
rosrun test_franka_gmp_common_bringup attempt_poses.sh
```

### 4. Collecting Training Data:

Download from [here](https://cf-my.sharepoint.com/:u:/g/personal/dumanf_cardiff_ac_uk/ETQ9KCFsV8dHuisX8GlsuKIB7GR7qZq0qPJE_v5a95YSQw?e=yOoNZD) 

Run (Terminal-1):

```bash
roscore
```

Run (Terminal-2):

```bash
rosrun robowflex_library run_franka.sh
```

Run (Terminal-3):

```bash
test-franka-gmp-desktop-robowflex
```

