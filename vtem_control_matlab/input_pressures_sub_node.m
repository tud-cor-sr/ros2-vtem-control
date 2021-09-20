% init workspace
% startup

% establish VtemControl class
global vtemControl
vtemControl = VtemControl(vtemDeviceAddress, vtemPort);
vtemControl.connect();

% ROS node initiated by Matlab
inputPressuresSubNode = ros2node("vtem_control/input_pressures_sub_node");
inputPressuresSub = ros2subscriber(inputPressuresSubNode, "/vtem_control/input_pressures", @input_pressures_callback);

function input_pressures_callback(msg)
    pressures = FluidPressures_msg_to_array(msg)*10^(-2); % [Pa] to [mBar]
    
    global vtemControl
    vtemControl.set_all_pressures(pressures);
end