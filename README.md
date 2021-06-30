# ROS2 VTEM Control
ROS2 package for the control of the Festo VTEM motion terminal. Adapted from https://github.com/tarquasso/3d_soft_trunk_contact/blob/master/SoftTrunk/src/MPA.cpp

## ROS2 package
### Build
```bash
colcon build --packages-select vtem_control
```
### Run

#### Run subscriber node to regulate pressure
```bash
ros2 run vtem_control input_pressures_sub_node
```
#### Run valve test
```bash
ros2 run vtem_control valve_test
```

### Run python publisher
```bash
python3 scripts/python_pub_node.py
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