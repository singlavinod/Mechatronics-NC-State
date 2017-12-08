%% ECE 556 Mechatronics PID control of autonomous robot 12/6/2017
% Task 1 & 2: Path tracking and Obstacle avoidance

clear all;      close all;	% housekeeping

% Initialize EV3
myev3 = legoev3('usb');

% Set proper port connections
l_motor_port = 'C'; % left motor port
r_motor_port = 'D'; % right motor port
l_sensor_port = 4; % left sensor port
r_sensor_port = 1; % right sensor port
ultra_sensor_port = 2; % ultrasonic sensor port

%%%%%%%%%%%%%%%%%%%%%%%% Light sensor calibration %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_cal = 100; % number of calibration samples
l_cal_w = zeros(1, n_cal); % left sensor, white surface
l_cal_b = zeros(1, n_cal); % left sensor, black surface
r_cal_w = zeros(1, n_cal); % right sensor, white surface
r_cal_b = zeros(1, n_cal); % right sensor, black surface

fprintf("Place the bot on white surface. Click anywhere on the figure when done.\n");
waitforbuttonpress;

for k = 1:n_cal
    l_cal_w(k) = readInputDeviceREADY_RAW(myev3,l_sensor_port,0,1);
    r_cal_w(k) = readInputDeviceREADY_RAW(myev3,r_sensor_port,0,1);
end

close all; % close the empty figure
fprintf("Place the bot on black surface. Click anywhere on the figure when done.\n");
waitforbuttonpress;

for k = 1:n_cal
    l_cal_b(k) = readInputDeviceREADY_RAW(myev3,l_sensor_port,0,1);
    r_cal_b(k) = readInputDeviceREADY_RAW(myev3,r_sensor_port,0,1);
end

close all; % close the empty figure

% Light sensor normalization constants, calibrate before each run
l_raw = [mean(l_cal_b) - 20, mean(l_cal_w) + 20];
r_raw = [mean(r_cal_b) - 20, mean(r_cal_w) + 20];
l_scale = 100/(l_raw(2) - l_raw(1));
r_scale = 100/(r_raw(2) - r_raw(1));

fprintf("Calibration for light sensors complete, place the bot on track.\n");
fprintf("Click anywhere on the figure when done.\n");
waitforbuttonpress;
close all; % close the empty figure

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ultrasonic sensor pre-calibrated between 3 and 60 cm
ultra_raw = [29 125]; % based on mean of 1000 readings
ultra_scale = (60 - 3)/(ultra_raw(2) - ultra_raw(1));

% Controller parameters
kt = 0.5; % tuning factor
kp = 0.7*kt; % proportional gain
kd = 0.02*kt; % derivative gain
rli_des = 50; % desired rli value
vs = 75; % straight path velocity
v = vs; % initial velocity
vc = 50; % curved path velocity
T = 0.004; % sample time for derivative control

% Read initial raw light sensor values
l_read_o = readInputDeviceREADY_RAW(myev3,l_sensor_port,0,1);
r_read_o = readInputDeviceREADY_RAW(myev3,r_sensor_port,0,1);
rli_l_o = l_scale*(l_read_o - l_raw(1));
rli_r_o = r_scale*(r_read_o - r_raw(1));

% Initialize error, commanded speeds and ultrasonic sensor reading
n = 2000; % set based on length of track
l_errs = zeros(1,n);
r_raw_read = zeros(1,n);
r_errs = zeros(1,n);
m_errs = zeros(1,n);
lmotor_v = zeros(1,n);
rmotor_v = zeros(1,n);
ultra_reads = zeros(1,n);
ultra_mv = zeros(1,n);
mv_err = zeros(1,n);
timestamp = zeros(1,n);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Run starting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf("Brace yourselves... \n");
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
    r_raw_read(i) = r_read; % anchor sensor reading for reporting
    rli_l = l_scale*(l_read - l_raw(1));
    rli_r = r_scale*(r_read - r_raw(1));
    
    % Ultrasonic reading and moving averages
    ultra_read = readInputDeviceREADY_RAW(myev3,ultra_sensor_port,0,1);
    ultra_reads(i) = ultra_read;
    
    if i > 4
        ultra_mv(i) = mean(ultra_reads(i - 4:i));
    else
        ultra_mv(i) = ultra_read;
    end
    
    % current distance of object from ev3 in cm
    dist_u = 3  + ultra_scale*(ultra_mv(i) - ultra_raw(1));
    
    %  Slow EV speed and stop based on the ultrasonic sensor reading
    if dist_u < 20
        vc = (dist_u - 15)*(vc - 20)/45 + 20;
        vs = (dist_u - 15)*(vs - 20)/45 + 20;
        if 3 > dist_u && 2 < dist_u
            disp('stop')
            lmotor.stop
            rmotor.stop
            break
        end
    else
        vc = 45;
        vs = 65;
    end
    
    % Calculate errors for control
    l_err = (rli_des - rli_l); % current iteration error
    l_err_o = (rli_des - rli_l_o); % previous iteration error
    
    r_err = (rli_des - rli_r); % current iteration error
    r_err_o = (rli_des - rli_r_o); % previous iteration error
    
    lc_out = kp*(l_err) + kd*(l_err - l_err_o)/T; % control action
    rc_out = kp*(r_err) + kd*(r_err - r_err_o)/T; % control action
    
    lmotor.Speed = v + lc_out - rc_out; % apply control
    rmotor.Speed = v - lc_out + rc_out; % apply control
    
    rli_l_o = rli_l; % reassign value for next iteration
    rli_r_o = rli_r; % reassign value for next iteration
    
    % Recordkeeping for analysis
    l_errs(i) = l_err;
    r_errs(i) = r_err;
    net_err = l_errs - r_errs;
    
    % Gain Scheduling based on curved or straight path
    if i > 12 % number of iteration found by trial & error
        mv_err(i) = mean(net_err(i - 12:i));
        if abs(mv_err(i)) > 55 % error value based on (0.5 to 1) feet curve
            kt = 0.4;
            kp = 0.7*kt;
            kd = 0.02*kt;
            v = vc;
        else
            kt = 0.3;
            kp = 0.7*kt;
            kd = 0.0*kt;
            v = vs;
        end
    end
    
    % Update timestamp
    timestamp_o = clock;
end
fprintf("Total time between start and end of run is given below. \n");
toc % end stopwatch

% stop bot if still running after n iterations
lmotor.stop;    rmotor.stop;

% Desired right sensor raw value corresponding to RLI 50
r_raw_des = r_raw(1) + 50/r_scale;
sprintf("Desired right raw sensor reading is \n: %4.2f", r_raw_des);

% Extract data at 50 ms intervals
y_in = r_raw_read(1:i);
t_in = timestamp;
t_in = t_in(1:i);
t_plot = 0.05.*(0:1:(i-1));
[raw_read, t] = resample(y_in, t_in, 20);
plot(t_plot(1:length(raw_read)), raw_read) % for reference
% Generate a txt file of errors
formatSpec = '@%4.2f@%1.2f\n';
filename = 'ECE556-group5_task1_run3.txt';
fileID = fopen(filename, 'w');
fprintf(fileID, formatSpec, [raw_read' t_plot(1:length(raw_read))']');
fclose(fileID);
hold on
plot(t_plot, y_in) % for reference
hold off;

% Plot raw value error
figure;
plot(t_plot(1:length(raw_read)), raw_read - r_raw_des);