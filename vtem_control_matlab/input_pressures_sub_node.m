% init workspace
% startup

% establish VtemControl class
global vtemControl
vtemControl = VtemControl(vtemDeviceAddress, vtemPort);
vtemControl.connect();

vtemControl.acknowledge_errors_all_slots();
vtemControl.activate_pressure_regulation_all_slots();

% ROS node initiated by Matlab
inputPressuresSubNode = ros2node("vtem_control/input_pressures_sub_node");
inputPressuresSub = ros2subscriber(inputPressuresSubNode, "/vtem_control/input_pressures", @input_pressures_callback);

vtemControl.deactivate_pressure_regulation_all_slots();

function input_pressures_callback(msg)
    pressures = FluidPressures_msg_to_array(msg)*10^(-2); % [Pa] to [mBar]
    
    global vtemControl
    vtemControl.set_all_pressures(pressures);
end