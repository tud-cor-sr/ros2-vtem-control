% init workspace
% init

% parameters
freq = 10; % [Hz]

% ROS node initiated by Matlab
outputPressuresPubNode = ros2node("vtem_control/output_pressures_pub_node");
vtemControlPressurePub = ros2publisher(outputPressuresPubNode, "output_pressures", "vtem_control_msgs/FluidPressures");

% establish VtemControl class
vtemControl = VtemControl(vtemDeviceAddress, vtemPort);
vtemControl.connect();

idx = 0;
while(1)
    idx = idx + 1;
    fprintf("Publish output pressures message %d \n", idx);
    
    pressures = vtemControl.get_all_pressures()*10^2; % [mBar] to [Pa]
    
    msg = array_to_FluidPressures_msg(pressures);
    
    % publish
    send(vtemControlPressurePub, msg)
    
    pause(1 / freq);
end