function pressures = FluidPressures_msg_to_array(msg)
    pressures = zeros(length(msg.data));
    for idx = 1:length(msg.data)
        pressures(idx) = msg.data(idx).fluid_pressure;
    end
end