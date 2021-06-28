# ROS2 VTEM Control
ROS2 package for the control of the Festo VTEM motion terminal. Adapted from https://github.com/tarquasso/3d_soft_trunk_contact/blob/master/SoftTrunk/src/MPA.cpp

## Build
```bash
colcon build --packages-select vtem_control
```

## Run
```bash
ros2 run vtem_control pressure_regulator_node
```