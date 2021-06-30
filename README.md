# ROS2 VTEM Control
ROS2 package for the control of the Festo VTEM motion terminal. Adapted from https://github.com/tarquasso/3d_soft_trunk_contact/blob/master/SoftTrunk/src/MPA.cpp

## ROS2 package
### Build
```bash
colcon build --packages-select vtem_control
```
### Run
```bash
ros2 run vtem_control pressure_regulator_node
```

## C++ package without ROS2 dependencies
### Build
```bash
cmake -S . -B build -DBUILD_WITH_ROS2=false
```
## Install
```bash
cmake --install build
```
### Run valve test
```bash
/usr/local/lib/vtem_control/valve_test
```