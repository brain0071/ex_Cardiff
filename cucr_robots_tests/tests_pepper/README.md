# CUCR Lab Demos (Pepper). How to Use:

These packages can be installed through [cucr-env](https://github.com/CardiffUniversityComputationalRobotics/tue-env).

Before installing any new package, update your installed packages:

```bash
cucr-get update
```

## Simulation tests

### 0. Installing (checking out) Pepper simulation tests

To install (check out) the Pepper tests in simulation:

```bash
cucr-get install ros-test_pepper_simulation_bringup
```

To build the new packages in either case:

```bash
cucr-make
```

### 1. MoveIt! tests:

To test the basic MoveIt! functionalities with Pepper, you should in separate terminals:

To start the Pepper in simulation (Gazebo):

```bash
test-pepper-simulation-full robot_env:=ipa-apartment moveit:=true
```

To start RViz:

```bash
test-pepper-desktop-rviz
```

Wait until the robot has put its arms down.

### 1.1 Using RViz visuals:

Once the simulation is running, check and activate the `MotionPlanning` display at the left of RViz. Use the visuals from RViz to move the arms to a desired position as below:

![moveit visuals](https://i.imgur.com/C6uqNzE.png)

Afterwards use the `Plan` and `Execute` buttons to move the Pepper's arms.

### 1.2 Using C++ script

Run the following command:

```bash
rosrun test_pepper_common_bringup pepper_moveit_demo
```

### 2. Mapping tests:

To test the basic `gmapping` functionalities with Pepper, you should in separate terminals:

To start the Pepper in simulation (Gazebo):

```bash
test-pepper-simulation-full robot_env:=ipa-apartment moveit:=true mapping:=true
```

To start RViz:

```bash
test-pepper-desktop-rviz
```

Once the simulation is running, wait till the robot puts its right arm at the front and the left arm behind before moving the base.

To teleoperate and move the base:

```bash
test-pepper-desktop-base-teleop
```

Start moving the robot around using this terminal and the map will start appearing in the RViz window.

Once all the mapping procedure is finished or it is desired to save the map, open a new terminal in a specific directory and run the following command without closing the simulation:

```bash
rosrun map_server map_saver -f {map_name}
```

> Note: `{map_name}` should be replaced. This will save two files, a `.pgm` and a `.yaml`, both are needed so that the map can be used afterwards.

### 3. Localization tests:

To test the basic `amcl` functionalities with Pepper, you should in separate terminals:

To start the Pepper in simulation (Gazebo):

```bash
test-pepper-simulation-full robot_env:=ipa-apartment moveit:=true localization:=true x_pose:=4 y_pose:=-4
```

> Note: To really test localization, spawn the robot in different locations by changing `x_pose` and `y_pose` parameters. To provide a custom map include the `map_file:={map_file_location}` parameter.

To start RViz:

```bash
test-pepper-desktop-rviz
```

To teleoperate and move the base:

```bash
test-pepper-desktop-base-teleop
```

Once the simulation is running completely, use the `2D Pose Estimate` tool located in the upper part of RViz and mark the estimated position of the robot with reference of the gazebo simulation.

Then start moving the robot around using the teleop window, eventually you will see the red particles around the robot converging to a single point in RViz meaning it has localized successfully.

### 4. Navigation tests:

To test the basic `move_base` functionalities with Pepper, you should in separate terminals:

To start the Pepper in simulation (Gazebo):

```bash
test-pepper-simulation-full robot_env:=ipa-apartment moveit:=true localization:=true navigation:=true x_pose:=4 y_pose:=-4
```

> Note: For navigation, localization is required. `DWAPlanner` is used with `move_base`. To provide a custom map include the `map_file:={map_file_location}` parameter.

To start RViz:

```bash
test-pepper-desktop-rviz
```

To teleoperate and move the base:

```bash
test-pepper-desktop-base-teleop
```

> Note: Close the teleop window once the robots is succesfully localized.

Use the `2D Nav Goal` tool in the upper side of RViz and set a goal for the robot, it should start moving towards the goal. It is also possible to localize the robot simply by defining goals but closing the teleop window is needed.

## Real-world tests

### 0. Installing (checking out) Pepper real-world tests

To install (check out) the Pepper robot tests in real-world:

```bash
cucr-get install ros-test_pepper_hardware_bringup
```

To build the new packages in either case:

```bash
cucr-make
```

Additionally, to control the real-world Pepper robot without errors, it is recommended to install naoqi the Python2.7 sdk.

```bash
echo "export PYTHONPATH=${PYTHONPATH}:/home/${USER}/ros/${ROS_DISTRO}/system/src/test_pepper_hardware_bringup/resources/pynaoqi-python2.7-2.5.7.1-linux64" >> ~/.bashrc
source ~/.bashrc
```

**Note:** for Noetic there is no naoqi sdk support, therefore you will get an error regarding the sdk, ignore it.

**Before starting the tests, assure that the Pepper robot is set and that it connects to the same network as the external computer to be used.**

### 1. MoveIt! tests:

To connect to the real-world robot and test the MoveIt! functionalities, run in separate terminals:

To start the connection with the real-world Pepper robot:

```bash
test-pepper-hardware-full nao_ip:=10.2.238.170 roscore_ip:=10.2.183.184 network_interface:=wlp0s20f3 moveit:=true
```

where the parameters correspond to:

- `nao_ip` - Pepper robot network IP.
- `roscore_ip` - ROS Master IP.
- `network_interface` - the external computer used interface to connect to the robot. Can be checked using `ifconfig`.

To start RViz:

```bash
test-pepper-desktop-rviz config:=real
```

Once RViz is running, check and activate the `MotionPlanning` display at the left of RViz. Use the visuals from RViz to move the arms to a desired position as shown in section 1.1 from simulation tests.

Afterwards use the `Plan` and `Execute` buttons to move the Pepper's arms.

Additionally, you can also teleoperate the robot:

```bash
test-pepper-desktop-base-teleop
```
