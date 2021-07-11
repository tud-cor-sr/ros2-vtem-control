function msg = array_to_FluidPressures_msg(array)
    msg = ros2message("vtem_control_msgs/FluidPressures");
    msg.header = ros2message("std_msgs/Header");
    % msg.Header.Stamp =
    
    fluidPressureBlankMsg = ros2message("sensor_msgs/FluidPressure");
    fluidPressureMsgs(length(array)) = fluidPressureBlankMsg;
    for idx = 1:length(array)
        fluidPressureMsgs(idx).header = ros2message("std_msgs/Header");
        fluidPressureMsgs(idx).fluid_pressure = array(idx);
        fluidPressureMsgs(idx).variance = 0;
    end
    
    msg.data = fluidPressureMsgs;
end