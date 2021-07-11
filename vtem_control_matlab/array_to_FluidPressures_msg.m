function msg = array_to_FluidPressures_msg(pressures)
    msg = ros2message("vtem_control_msgs/FluidPressures");
    msg.header = ros2message("std_msgs/Header");
    
    fluidPressureBlankMsg = ros2message("sensor_msgs/FluidPressure");
    fluidPressureMsgs(length(pressures)) = fluidPressureBlankMsg;
    for idx = 1:length(pressures)
        fluidPressureMsgs(idx).header = ros2message("std_msgs/Header");
        fluidPressureMsgs(idx).fluid_pressure = pressures(idx);
        fluidPressureMsgs(idx).variance = 0;
    end
    
    msg.data = fluidPressureMsgs;
end