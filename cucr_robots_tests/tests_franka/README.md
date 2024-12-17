# CUCR Lab Demos (Franka). How to Use:

These packages can be installed through [tue-env](https://github.com/CardiffUniversityComputationalRobotics/tue-env).

## Simulation tests

### 0. Installing (checking out) Franka simulation tests

Before installing any new package, update your installed packages:

```bash
cucr-get update
```

To install (check out) the Franka tests in simulation:

```bash
cucr-get install ros-test_franka_simulation_bringup
```

To build the new packages in either case:

```bash
cucr-make
```

Source updated setup files:

```bash
source ~/.bashrc
```

### 1. MoveIt! tests:

To test the basic MoveIt! functionalities with Franka

To start the Franka in simulation (Gazebo) (Terminal-1):

```bash
test-franka-simulation-full
```

To start RViz (Terminal-2):

```bash
test-franka-desktop-rviz
```

:warning: **Warning** :warning: The automated installation of the target test_franka_simulation_bringup will add some env variables to `~/.bashrc`, which might affect the proper operation/functioning of Gazebo. If that is the case, you can edit `~/.bashrc` and comment out those env variables export, and launch a new terminal to use Gazebo.

#### 1.1 Using RViz visuals:

Once the simulation is running, check and activate the `MotionPlanning` display at the left of RViz. Use the visuals from RViz to move the arms to a desired position as below:

![image](https://user-images.githubusercontent.com/98045613/192559753-94cbf399-7920-4a0c-8c89-405eb7dd608f.png)

Afterwards use the `Plan` and `Execute` buttons to move the Franka's arms.

#### 1.2 Using C++ script (Pick and Place Demo)

In this demo, move function is used for pick and place. Run the following command (Terminal-3):

```bash
rosrun test_franka_common_bringup franka_move
```

or

In this demo, pick function is used for pick and place. Run the following command (Terminal-3):

```bash
rosrun test_franka_common_bringup franka_pick
```

### 2. Using Python script in CoppeliaSim(V-Rep):

```bash
rosrun test_franka_common_bringup vrep_demo.py
```


## Real-world tests

### 0. Installing (checking out) Franka real-world tests

To install (check out) the Franka robot tests in real-world:

```bash
cucr-get install ros-test_franka_hardware_bringup
```

To build the new packages in either case:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```

Source updated setup files:

```bash
source ~/.bashrc
```
To install real-time kernel for Franka:


Check the correct version for Franka [Compatible versions](https://frankaemika.github.io/docs/compatibility.html).


Install the real-time kernel [real-time kernel](https://frankaemika.github.io/docs/installation_linux.html).

### 1. MoveIt! tests:

To start the Franka in real (Terminal-1):

```bash
test-franka-hardware-full robot_ip:=172.16.0.2 robot:=fr3 arm_id:=fr3
```

To load obstacles for the Franka in real (Terminal-2):

```bash
rosrun test_franka_hardware_bringup franka_real_demo
```






