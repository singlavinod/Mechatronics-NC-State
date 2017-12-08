%% ECE 556 Mechatronics PID control of autonomous robot 12/6/2017
% Task 3: Platooning
clear all
close all
% Initialize EV3
myev3 = legoev3('usb');

% Set proper port connections
l_motor_port = 'C'; % left motor port
r_motor_port = 'D'; % right motor port
l_sensor_port = 4; % left sensor port
r_sensor_port = 1; % right sensor port
m_sensor_port = 3; % right sensor port
ultra_sensor_port = 2; % ultrasonic sensor port

% %%%%%%%%%%%%%%%%%%%%%%% Light sensor calibration %%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_cal = 100; % number of calibration samples
l_cal_w = zeros(1, n_cal); % left sensor, white surface
l_cal_b = zeros(1, n_cal); % left sensor, black surface
r_cal_w = zeros(1, n_cal); % right sensor, white surface
r_cal_b = zeros(1, n_cal); % right sensor, black surface
m_cal_w = zeros(1, n_cal); % middle sensor, white surface
m_cal_b = zeros(1, n_cal); % middle sensor, black surface

fprintf("Place the bot on white surface. Click anywhere on the figure when done.\n");
waitforbuttonpress;

for k = 1:n_cal
    l_cal_w(k) = readInputDeviceREADY_RAW(myev3,l_sensor_port,0,1);
    r_cal_w(k) = readInputDeviceREADY_RAW(myev3,r_sensor_port,0,1);
    m_cal_w(k) = readInputDeviceREADY_RAW(myev3,m_sensor_port,0,1);
end

close all; % close the empty figure
fprintf("Place the bot on black surface. Click anywhere on the figure when done.\n");
waitforbuttonpress;

for k = 1:n_cal
    l_cal_b(k) = readInputDeviceREADY_RAW(myev3,l_sensor_port,0,1);
    r_cal_b(k) = readInputDeviceREADY_RAW(myev3,r_sensor_port,0,1);
    m_cal_b(k) = readInputDeviceREADY_RAW(myev3,m_sensor_port,0,1);
end

close all; % close the empty figure

% Light sensor normalization constants, calibrate before each run
l_raw = [mean(l_cal_b) - 20, mean(l_cal_w) + 20];
r_raw = [mean(r_cal_b) - 20, mean(r_cal_w) + 20];
m_raw = [mean(m_cal_b) - 20, mean(m_cal_w) + 20];

l_scale = 100/(l_raw(2) - l_raw(1));
r_scale = 100/(r_raw(2) - r_raw(1));
m_scale = 100/(m_raw(2) - m_raw(1));

close all; % close the empty figure
fprintf("Place the middle sensor on red. Click anywhere on the figure when done.\n");
waitforbuttonpress;

for k = 1:n_cal
    m_cal_r(k) = readInputDeviceREADY_RAW(myev3,m_sensor_port,0,1);
    rli_cal_m(k) = m_scale*(m_cal_r(k) - m_raw(1));
end

rli_stop = mean(rli_cal_m);

fprintf("Calibration for light sensors complete, place the bot on track.\n");
fprintf("Click anywhere on the figure when done.\n");
waitforbuttonpress;
close all; % close the empty figure

% Ultrasonic sensor pre-calibrated between 3 and 60 cm
ultra_raw = [29 125]; % based on mean of 1000 readings
ultra_scale = (60 - 3)/(ultra_raw(2) - ultra_raw(1));

% Controller parameters, path tracking
kt = 0.2; % tuning factor
kp = 0.7*kt; % proportional gain
kd = 0.02*kt; % derivative gain
rli_des = 50; % desired rli value
vs = 100; % straight path velocity
v = vs; % initial velocity
vc = 45; % curved path velocity
T = 0.004; % sample time for derivative control

% Controller parameters, platooning
sep_des = 20; % desired separation

% Red stop sensor rli values
rli_stop_u = rli_stop; % Actual upper = 76
rli_stop_l = rli_stop - 5; % Actual lower = 75;

% Run parameters
n = 2000; % number of samples to run for

% Read initial raw sensor and distance values
l_read_o = readInputDeviceREADY_RAW(myev3,l_sensor_port,0,1);
r_read_o = readInputDeviceREADY_RAW(myev3,r_sensor_port,0,1);
rli_l_o = l_scale*(l_read_o - l_raw(1));
rli_r_o = r_scale*(r_read_o - r_raw(1));
ultra_read_o = readInputDeviceREADY_RAW(myev3,ultra_sensor_port,0,1);
dist_u_o = 3  + ultra_scale*(ultra_read_o - ultra_raw(1));
sep_err_o = 20;

% Initialize error, commanded speeds and ultrasonic sensor reading
l_errs = zeros(1,n);
r_raw_read = zeros(1,n);
r_errs = zeros(1,n);
m_errs = zeros(1,n);
lmotor_v = zeros(1,n);
rmotor_v = zeros(1,n);
ultra_reads = zeros(1,n);
ultra_mv = zeros(1,n);
timestamp = zeros(1,n);
dist_us = zeros(1,n);

% Initialize EV3
lmotor = motor(myev3,l_motor_port);
rmotor = motor(myev3,r_motor_port);
start(lmotor);
start(rmotor);
lmotor.Speed = v;
rmotor.Speed = v;

tic % start stopwatch
timestamp_o = clock; % initialize timestamp
offset = timestamp_o(6); % used for post-processing;

for i = 1:n
    % Record timestamp
    timestamp(i) = timestamp_o(6);
    
    % Read current raw light sensor values
    l_read = readInputDeviceREADY_RAW(myev3,l_sensor_port,0,1);
    r_read = readInputDeviceREADY_RAW(myev3,r_sensor_port,0,1);
    m_read = readInputDeviceREADY_RAW(myev3,m_sensor_port,0,1);
    rli_l = l_scale*(l_read - l_raw(1));
    rli_r = r_scale*(r_read - r_raw(1));
    rli_m = m_scale*(m_read - m_raw(1));
    
    % Ultrasonic reading and moving averages
    ultra_read = readInputDeviceREADY_RAW(myev3,ultra_sensor_port,0,1);
    ultra_reads(i) = ultra_read;
    
    if i > 4
        ultra_mv(i) = mean(ultra_reads((i - 4):i));
    else
        ultra_mv(i) = ultra_read;
    end
    
    dist_u = 3  + ultra_scale*(ultra_mv(i) - ultra_raw(1));
    dist_us(i) = dist_u;
    
    %  follow based on the ultrasonic sensor
    sep_err = sep_des - dist_u;
    
    % Calculate errors
    l_err = (rli_des - rli_l);
    l_err_o = (rli_des - rli_l_o);
    
    r_err = (rli_des - rli_r); % current iteration error
    r_err_o = (rli_des - rli_r_o); % previous iteration error
    
    lc_out = kp*(l_err) + kd*(l_err - l_err_o)/T; % control action
    rc_out = kp*(r_err) + kd*(r_err - r_err_o)/T; % control action
    v = min(-sep_err*vs/10, vs);
    lmotor.Speed = max(0, v + lc_out - rc_out);
    rmotor.Speed = max(0, v - lc_out + rc_out);
    
    rli_l_o = rli_l;
    rli_r_o = rli_r;
    sep_err_o = sep_err;
    dist_u_o = dist_u;
    
    %  Stop based on the center sensor
    if rli_stop_u > rli_m && rli_stop_l < rli_m
        disp('stop')
        lmotor.stop
        rmotor.stop
        break
    end
    
    % Update timestamp
    timestamp_o = clock;
    
end
fprintf("Total time between start and end of run is given below. \n");
toc % end stopwatch

% stop bot if still running after n iterations
lmotor.stop;    rmotor.stop;

% Extract data at 50 ms intervals
y_in = dist_us(1:i);
t_in = timestamp;
t_in = t_in(1:i);
t_plot = 0.05.*(0:1:(i-1));
[dist_read, t] = resample(y_in, t_in, 20);
% plot(t_plot(1:length(dist_read)), dist_read) % for reference
% Generate a txt file of errors
formatSpec = '@%4.2f@%1.2f\n';
filename = 'ECE556-group5_task3_run2.txt';
fileID = fopen(filename, 'w');
fprintf(fileID, formatSpec, [dist_read' t_plot(1:length(dist_read))']');
fclose(fileID);
hold on;
plot(t_plot, y_in) % for reference