% Adapted from:
% https://www.mathworks.com/help/ros/ug/get-started-with-ros-2.html

# sample message
msgBlank = ros2message("vtem_control/msg/InputPressures")
msgBlank.data = [1, 2, 0.5]*10^5 % [Pa]

% ROS node initiated by Matlab
srControlNode = ros2node("/sr_control");
vtemControlPressurePub = ros2publisher(srControlNode, "/vtem_control/input_pressures", "vtem_control/msg/InputPressures");
msgs(10) = msgBlank;    % Pre-allocate message structure array
for idx = 1:10
    % Copy blank message fields
    msgs(idx) = msgBlank;
    
    % Record sample temperature
    msgs(idx).data = [randn, randn, randn]*10^5 % [Pa]
    
    % Pass the data to subscribers
    send(vtemControlPressurePub, msgs(idx))
end