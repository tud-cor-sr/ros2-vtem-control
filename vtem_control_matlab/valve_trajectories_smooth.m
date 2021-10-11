% close all; clear;

% parameters
nchambers = 4;

vtem_control = VtemControl("192.168.4.3", 502, nchambers);
vtem_control.connect();

% Acknowledge errors
vtem_control.acknowledge_errors_all_slots();

if vtem_control.activate_pressure_regulation_all_slots() == false
    throw(MException("valve_test:activate_pressure_regulation_single_slot", "Failed to activate pressure regulation."))
end

%% parameters
rp = 0.1; % distance between center of pressure force and the axis
alpha_p = 100; % maps pressure to force. 1mbar = 100N/m2
alpha = rp * alpha_p;% parameter to be assigned
p_offset = 150; % preload pressure value in one chamber

%% Trajectories
trajectory_number = 3;

% Trajectory 1: 1D bending
if trajectory_number == 1
    slope = 600000; % 32 time steps instead of 38, around 10s traj
    force_peak = 3000; % [N] 
    up = 0:slope/force_peak:force_peak;
    down = force_peak:-slope/force_peak:0;
    f0 = [up, down];
    f1 = zeros(1,length(f0));

% Trajectory 2: a half-8-shape
elseif trajectory_number == 2
    slope = 300000; %32 steps, 10s
    force_peak = 1500; % [N]
    up = 0:slope/force_peak:force_peak;
    down = force_peak:-slope/force_peak:0;
    f0 = [up, down, -up, -down];
    f1 = [up, up + force_peak, down + force_peak, down];
    
% Trajectory 3: a (full) 8-shape
elseif trajectory_number == 3
    slope = 600000; %32 steps, 10s
    force_peak = 1500; % [N]
    up = 0:slope/force_peak:force_peak;
    down = force_peak:-slope/force_peak:0;
    f0 = [up, down, -up, -down, up, down, -up, -down];
    f1 = [up, up + force_peak, down + force_peak, down, -up, -up - force_peak, -down - force_peak, -down];
end


%% four chamber case
if nchambers == 4
    pp = 1 / alpha * [1/2 0; 0 1/2; -1/2 0; 0 -1/2] * [f0;f1] + p_offset;
    p0 = pp(1,:) - min([zeros(1,length(pp));pp(1,:);pp(3,:)]);
    p1 = pp(2,:) - min([zeros(1,length(pp));pp(2,:);pp(4,:)]);
    p2 = pp(3,:) - min([zeros(1,length(pp));pp(1,:);pp(3,:)]);
    p3 = pp(4,:) - min([zeros(1,length(pp));pp(2,:);pp(4,:)]);
    % Trying to smooth the pressure profiles 
    p0 = smooth(p0);
    p1 = smooth(p1);
    p2 = smooth(p2);
    p3 = smooth(p3);
    
    disp('peak pressures [mBar]: ');
    disp([max(p0) max(p1) max(p2) max(p3)]);
    disp('time steps: ');
    disp(length(pp));
    disp('pressure offset [mBar]: ');
    disp(p_offset);
    disp('delta pressure [mBar]: ');
    disp([max(p0)-min(p0) max(p1)-min(p1) max(p2)-min(p2) max(p3)-min(p3)]);
    
elseif nchambers == 3
%% three chamber case
    pp = 1 / alpha * [2/3 0; -1/3 1/sqrt(3); -1/3 -1/sqrt(3)] * [f0;f1] + p_offset;
    p0 = pp(1,:) - min([zeros(1,length(pp));pp(1,:);pp(2,:);pp(3,:)]);
    p1 = pp(2,:) - min([zeros(1,length(pp));pp(1,:);pp(2,:);pp(3,:)]);
    p2 = pp(3,:) - min([zeros(1,length(pp));pp(1,:);pp(2,:);pp(3,:)]);
end

subplot(4,1,1); plot(p0); title('chamber 1 desired pressure values'); ylabel('pressure [mBar]'); xlabel('time steps');
subplot(4,1,2); plot(p1); title('chamber 2 desired pressure values'); ylabel('pressure [mBar]'); xlabel('time steps');
subplot(4,1,3); plot(p2); title('chamber 3 desired pressure values'); ylabel('pressure [mBar]'); xlabel('time steps');
if nchambers == 4
    subplot(4,1,4); plot(p3); title('chamber 4 desired pressure values'); ylabel('pressure [mBar]'); xlabel('time steps');
end

% suptitle('trajectory 1 desired pressure values');
% figure; plot(f0,f1); xlabel('f0 [N]'), ylabel('f1 [N]'); title('forces in x and y');

x = zeros(length(pp),16);

vtem_control.set_all_pressures([round(1*p_offset/4)*ones(4,1);zeros(12,1)]);
pause(0.3);
vtem_control.set_all_pressures([round(2*p_offset/4)*ones(4,1);zeros(12,1)]);
pause(0.3);
vtem_control.set_all_pressures([round(3*p_offset/4)*ones(4,1);zeros(12,1)]);
pause(0.3);
vtem_control.set_all_pressures([4*p_offset/4*ones(4,1);zeros(12,1)]);

%vtem_control.set_all_pressures(zeros(16,1));
while vtem_control.get_single_pressure(0) < 147 && vtem_control.get_single_pressure(1) < 147 && vtem_control.get_single_pressure(2) < 147 && vtem_control.get_single_pressure(valveIdx+3) < 147
    disp('not ready yet')
end
pause(5);
tic
for i=1:1:length(pp)
    x(i,:) =  (vtem_control.get_all_pressures)';
    
    vtem_control.set_single_pressure(0, round(p0(i))); % here you set pressure!
    vtem_control.set_single_pressure(1, round(p1(i)));
    vtem_control.set_single_pressure(2, round(p2(i)));
    if nchambers == 4
        vtem_control.set_single_pressure(3, round(p3(i)));
    end
    pause(0.1); % 10Hz
end
toc

vtem_control.set_all_pressures([round(3*p_offset/4)*ones(4,1);zeros(12,1)]);
pause(0.3);
vtem_control.set_all_pressures([round(2*p_offset/4)*ones(4,1);zeros(12,1)]);
pause(0.3);
vtem_control.set_all_pressures([round(1*p_offset/4)*ones(4,1);zeros(12,1)]);
pause(0.3);

% deactivate pressure regulation
vtem_control.deactivate_pressure_regulation_all_slots();

figure; 
subplot(2,2,1); plot(x(1:end,1)); hold on; plot(p0'); legend('read values','desired values'); xlabel('time steps'); ylabel('pressures [mBar]'); title('chamber 1');
subplot(2,2,2); plot(x(1:end,2)); hold on; plot(p1'); legend('read values','desired values'); xlabel('time steps'); ylabel('pressures [mBar]'); title('chamber 2');
subplot(2,2,3); plot(x(1:end,3)); hold on; plot(p2'); legend('read values','desired values'); xlabel('time steps'); ylabel('pressures [mBar]'); title('chamber 3');
subplot(2,2,4); plot(x(1:end,4)); hold on; plot(p3'); legend('read values','desired values'); xlabel('time steps'); ylabel('pressures [mBar]'); title('chamber 4');
% title('comparison between desired and actual pressure values');
