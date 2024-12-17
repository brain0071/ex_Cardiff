# CUCR Lab Demos (Unitree A1). How to Use:

These packages can be installed through [cucr-env](https://github.com/CardiffUniversityComputationalRobotics/tue-env).

Before installing any new package, update your installed packages:

```bash
cucr-get update
```

## Simulation tests

### 0. Installing (checking out) Unitree A1 simulation tests

To install (check out) the Unitree A1 tests in simulation:

```bash
cucr-get install ros-test_a1_simulation_bringup
ros/noetic/system/src/cartographer/scripts/install_abseil.sh
```

To build the new packages in either case:

```bash
cucr-make
```

### 1. Mapping tests:

For this specific robot, we have decided to use [Cartographer ROS](https://github.com/cartographer-project/cartographer_ros). It is a real-time SLAM package, which means it can perform mapping and localization together. This package is able to use several inputs like pointclouds, IMUs and odometry. For this particular demo, you can decide whether or not to use odometry.

To start the Unitree A1 in simulation (Gazebo):

```bash
test-a1-simulation-full mapping:=true use_odometry:=true
```

To start RViz:

```bash
test-a1-desktop-rviz
```

To teleoperate and move the base:

```bash
test-a1-desktop-base-teleop
```

> Note: it is recommended to lower the velocity to 0.2 m/s to have better mapping results.

![](https://i.imgur.com/RRzgZ8G.png)

Start moving the robot around using the teleop terminal and the map will start appearing in the RViz window.
![](https://i.imgur.com/LyJCe23.png)

Once all the mapping procedure is finished, to save the map in order to perform localization with Cartographer ROS, run the following commands in a new terminal:

```bash
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: 'path/to/file.bag.pbstream'}"
```

**Example filename:** `/home/sasm/ros/noetic/system/src/test_a1_simulation_bringup/maps/construction_map.bag.pbstream`

This command will save a `.pbstream` file which can be used by used to have localization only with Cartographer ROS. You could also save the common `.pgm` and `.yaml` files using [map_server](http://wiki.ros.org/map_server).

### 2. Localization tests:

To test the Cartographer ROS localization functionalities with Unitree A1, you should in separate terminals:

To start the Unitree A1 in simulation (Gazebo):

```bash
test-a1-simulation-full localization:=true use_odometry:=true x_pose:=2 y_pose:=2
```

> Note: To really test localization, spawn the robot in different locations by changing `x_pose` and `y_pose` parameters. To provide a custom pbstream map file include the `map_pbstream_file:={map_file_location}` parameter.

To start RViz:

```bash
test-a1-desktop-rviz
```

To teleoperate and move the base:

```bash
test-a1-desktop-base-teleop
```

Once the simulation is running completely, start moving the robot around using the teleop window, while the robot is not localized, Cartographer ROS will start updating the given map with the robot's current position. When Cartographer ROS localizes the robot, it will correct the robot's position and revert the changes to the given map.

Unfortunately, Cartographer ROS does not give any feedback about the localization process as [amcl](http://wiki.ros.org/amcl) does.

### 3. Navigation tests:

To test the basic `move_base` functionalities with Unitree A1, you should in separate terminals:

To start the Unitree A1 in simulation (Gazebo):

```bash
test-a1-simulation-full navigation:=true use_odometry:=true x_pose:=2 y_pose:=2
```

> Note: `DWAPlanner` is used along with `move_base` to perform navigation. To provide a custom map include the `map_pbstream_file:={map_file_location}` parameter.

To start RViz:

```bash
test-a1-desktop-rviz
```

To teleoperate and move the base to localize the robot:

```bash
test-a1-desktop-base-teleop
```

> Note: Close the teleop window once the robots is succesfully localized.

Finally, use the `2D Nav Goal` tool in the upper side of RViz and set a goal for the robot, it should start moving towards the goal. It is also possible to localize the robot without teleoperating and simply defining goals from the start.

## Real-world tests

### 0. Installing (checking out) Unitree A1 real-world tests

To install (check out) the Unitree A1 tests in real-world:

```bash
cucr-get install ros-test_a1_hardware_bringup
```

To build the new packages in either case:

```bash
cucr-make
```

Before installing any new package, update your installed packages:

```bash
cucr-get update
```

### 1. Connect to robot

To control the A1 robot and retrieve its sensor data, first you must connect to the robot using WiFi. For that follow the instructions below;

1. Position the robot laying down correctly on the floor.
2. Double tap and hold the power on button.
3. Wait until robot stands up and connect to the robot's access point. SSID: `UnitreeRoboticsA1-XXX`. Password: `00000000`.

### 2. Local localization tests:

To test the localization functionality of the real-world Unitree A1 by using the IMU and its own legs' movement, you should in separate terminals:

To start the connection with the real-world Unitree A1:

```bash
test-a1-hardware-full
```

To start RViz:

```bash
test-a1-desktop-rviz config:=real
```

To teleoperate the robot:

```bash
test-a1-desktop-base-teleop cmd_vel:=/cmd_vel
```

Move the robot around and check how in RViz its position is tracked.

To activate the robot's depth camera and retrieve its data, follow these steps:

1. First connect directly to the robot computer using ssh:

```bash
ssh pi@192.168.123.12 # password: "raspberry"
```

2. Modify last line of `.bashrc` as follows:

```bash
export ROS_MASTER_URI=http://{ROS_MASTER_IP}:11311
```

3. Stop any vision program using the camera:

```bash
sudo pkill -f Vision
```

4. Run the camera node:

```bash
roslaunch realsense2_camera demo_pointcloud.launch
```

**Note:** you may have timing issues, if so check the following link [ROS Network Setup](http://wiki.ros.org/ROS/NetworkSetup) and configure `chrony`.
