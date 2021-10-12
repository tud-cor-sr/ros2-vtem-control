% deactivate pressure regulation
if vtem_control.deactivate_pressure_regulation_all_slots() == false
    throw(MException("valve_test:deactivate_pressure_regulation_single_slot", "Failed to deactivate pressure regulation."))
end

% disconnect
vtem_control.disconnect();