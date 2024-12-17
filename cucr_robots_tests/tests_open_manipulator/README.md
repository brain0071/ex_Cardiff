# CUCR Lab Demos (Open Manipulator). How to Use:

These packages can be installed through [cucr-env](https://github.com/CardiffUniversityComputationalRobotics/tue-env).

Before installing any new package, update your installed packages:

```bash
cucr-get update
```

## Simulation tests

### 0. Installing (checking out) Open Manipulator simulation tests

To install (check out) the Open Manipulator tests in simulation:

```bash
cucr-get install ros-test_open_manipulator_simulation_bringup
```

To build the new packages in either case:

```bash
cucr-make
```

### 1. Moveit tests:

To start the Open Manipulator in simulation (Gazebo):

```bash
test-open-manipulator-simulation-full
```

To start RViz:

```bash
test-open-manipulator-desktop-rviz
```
