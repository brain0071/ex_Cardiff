# CUCR Lab Demos (TIAGo). How to Use:

These packages can be installed through [cucr-env](https://github.com/CardiffUniversityComputationalRobotics/tue-env).

Before installing any new package, update your installed packages:

```bash
cucr-get update
```

## Simulation tests

### 0. Installing (checking out) TIAGo simulation tests

To install (check out) the TIAGo tests in simulation:

```bash
cucr-get install ros-test_tiago_simulation_bringup
```

:warning: **Note**: have in mind that to use TIAGo in simulation, this installation will overlay the packages in [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs), [ros_control](https://github.com/ros-controls/ros_control) and [ros_controllers](https://github.com/ros-controls/ros_controllers). This is highly undesired but required. If you are not going to use TIAGo, you can just remove these folders from the source directory in the workspace.

To build the new packages in either case:

```bash
cucr-make
```

### 1. MoveIt! tests:

To test the basic MoveIt! functionalities with TIAGo, you should in separate terminals:

To start the TIAGo in simulation (Gazebo):

```bash
test-tiago-simulation-full
```

To start RViz:

```bash
test-tiago-desktop-rviz config:=odom
```

Wait until the robot has put its arms in a tuck position.

### 1.1 Using RViz visuals:

Once the simulation is running, use the visuals from RViz to move the arms to a desired position as below:

![moveit visuals](https://i.imgur.com/TDmLpOM.png)

Afterwards use the `Plan` and `Execute` buttons to move the TIAGo's arms.

### 1.2 Using C++ script

Run the following command:

```bash
rosrun test_tiago_common_bringup tiago_moveit_demo
```

### 2. Mapping tests:

To test the basic `gmapping` functionalities with TIAGo, you should in separate terminals:

To start the TIAGo in simulation (Gazebo):

```bash
test-tiago-simulation-full mapping:=true
```

To start RViz:

```bash
test-tiago-desktop-rviz
```

Once the simulation is running, wait till the robot puts its arm in a tuck position.

To teleoperate and move the base:

```bash
test-tiago-desktop-base-teleop
```

Start moving the robot around using this terminal and the map will start appearing in the RViz window.

Once all the mapping procedure is finished or it is desired to save the map, open a new terminal in a specific directory and run the following command without closing the simulation:

```bash
rosrun map_server map_saver -f {map_name}
```

> Note: `{map_name}` should be replaced. This will save two files, a `.pgm` and a `.yaml`, both are needed so that the map can be used afterwards.

### 3. Localization tests:

To test the basic `amcl` functionalities with TIAGo, you should in separate terminals:

To start the TIAGo in simulation (Gazebo):

```bash
test-tiago-simulation-full localization:=true x_pose:=1 y_pose:=-1
```

> Note: To really test localization, spawn the robot in different locations by changing `x_pose` and `y_pose` parameters. To provide a custom map include the `map_file:={map_file_location}` parameter.

To start RViz:

```bash
test-tiago-desktop-rviz
```

To teleoperate and move the base:

```bash
test-tiago-desktop-base-teleop
```

Once the simulation is running completely, use the `2D Pose Estimate` tool located in the upper part of RViz and mark the estimated position of the robot with reference of the gazebo simulation.

Then start moving the robot around using the teleop window, eventually you will see the red particles around the robot converging to a single point in RViz meaning it has localized successfully.

### 4. Navigation tests:

To test the basic `move_base` functionalities with TIAGo, you should in separate terminals:

To start the TIAGo in simulation (Gazebo):

```bash
test-tiago-simulation-full localization:=true navigation:=true x_pose:=1 y_pose:=-1
```

> Note: For navigation, localization is required. `DWAPlanner` is used with `move_base`. To provide a custom map include the `map_file:={map_file_location}` parameter.

To start RViz:

```bash
test-tiago-desktop-rviz
```

To teleoperate and move the base:

```bash
test-tiago-desktop-base-teleop
```

> Note: Close the teleop window once the robots is succesfully localized.

Use the `2D Nav Goal` tool in the upper side of RViz and set a goal for the robot, it should start moving towards the goal. It is also possible to localize the robot simply by defining navigation goals.

## Real-world tests

### 0. Installing (checking out) TIAGo real-world tests

TODO

### 1. MoveIt! tests:

TODO
