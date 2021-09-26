% parameters
valveNum = 0; % test valves 0 to 15
commandPressure = 450; % [mBar]
endPressure = 0;

vtem_control = VtemControl("192.168.1.2", 502);
vtem_control.connect();
vtem_control.activate_pressure_regulation_all_slots();

vtem_control.set_single_pressure(valveNum, commandPressure);

num_cycles = 1000;
t = 1:1:num_cycles;
x = zeros(num_cycles, 1);
x_des = zeros(num_cycles, 1);
for i=1:1:num_cycles
    x(i) = vtem_control.get_single_pressure(valveNum);
    
    x_des(i) = step_func(i);
    vtem_control.set_single_pressure(valveNum, x_des(i));
end

vtem_control.set_single_pressure(valveNum, endPressure);

figure
plot(t,x,t,x_des);

function value = step_func(i)
    if mod(i, 1500) < 750
        value = 300; % [mBar]
    else
        value = 450; % [mBar]
    end
end