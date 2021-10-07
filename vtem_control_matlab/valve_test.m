% parameters
commandPressure = 450; % [mBar]
endPressure = 0;

valveIdx = 0; % test valves 0 to 15
[slotIdx, slotRemain] = VtemControl.get_slot_idx_from_valve_idx(valveIdx);

vtem_control = VtemControl("192.168.4.3", 502);
vtem_control.connect();

% Acknowledge errors
vtem_control.acknowledge_errors_single_slot(slotIdx);

if vtem_control.activate_pressure_regulation_single_slot(slotIdx) == false
    throw(MException("valve_test:activate_pressure_regulation_single_slot", "Failed to activate pressure regulation."))
end

num_cycles = 100;
t = 1:1:num_cycles;
x = zeros(num_cycles, 1);
x_des = zeros(num_cycles, 1);
% tic
for i=1:1:num_cycles
    x(i) = vtem_control.get_single_pressure(valveIdx);
    
    x_des(i) = step_func(i);
    vtem_control.set_single_pressure(valveIdx, x_des(i));

    % toc
    
    pause(0.1);
end

if vtem_control.deactivate_pressure_regulation_single_slot(slotIdx) == false
    throw(MException("valve_test:deactivate_pressure_regulation_single_slot", "Failed to deactivate pressure regulation."))
end

figure
plot(t,x,t,x_des);

function value = step_func(i)
    if mod(i, 1500) < 750
        value = 300; % [mBar]
    else
        value = 450; % [mBar]
    end
end