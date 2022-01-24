# ROS2 VTEM Control

ROS2 package for the control of the Festo VTEM motion terminal. Adapted from https://github.com/tarquasso/3d_soft_trunk_contact/blob/master/SoftTrunk/src/MPA.cpp

## ROS2 package

### Build

```bash
colcon build --packages-select vtem_control_cpp vtem_control_msgs
```

### Run

#### Run input and output pressures nodes using launch file

```bash
ros2 launch vtem_control_cpp vtem_control.launch.py
```

#### Run subscriber node to send input pressures to VTEM

```bash
ros2 run vtem_control_cpp input_pressures_sub_node
```

#### Run publisher node to grab output pressures from VTEM

```bash
ros2 run vtem_control_cpp output_pressures_pub_node
```

#### Run valve test

```bash
ros2 run vtem_control_cpp valve_test
```

#### Run python publisher for input pressure step function

```bash
python3 scripts/python_pub_node.py
```

## C++ package without ROS2 dependencies

### Build

```bash
cmake -S . -B build -DBUILD_WITH_ROS2=false
```

### Install

```bash
cmake --install build
```

### Run valve test

```bash
/usr/local/lib/vtem_control_cpp/valve_test
```

## Matlab

Requires Python 3.7 to be installed to load custom FluidPressures messages
