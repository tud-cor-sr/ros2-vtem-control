% Adapted from:
% https://www.mathworks.com/help/ros/ug/get-started-with-ros-2.html

% init workspace
% startup

% parameters
freq = 10; % [Hz]

% ROS node initiated by Matlab
inputPressuresPubNode = ros2node("vtem_control/input_pressures_pub_node");
inputPressuresPub = ros2publisher(inputPressuresPubNode, "/vtem_control/input_pressures", "vtem_control_msgs/FluidPressures");

% some test messages
for idx = 1:1000
    fprintf("Publish input pressures message %d \n", idx);
    pressures = zeros(16, 1);
    pressure = step_func(idx);
    pressures(1) = pressure;
    
    msg = array_to_FluidPressures_msg(pressures);
    
    % Pass the data to subscribers
    send(inputPressuresPub, msg)
    
    pause(1/freq);
end


function value = step_func(i)
    if mod(i, 1500) < 750
        value = 0.30*10^5; % [mBar]
    else
        value = 0.45*10^5; % [mBar]
    end
end