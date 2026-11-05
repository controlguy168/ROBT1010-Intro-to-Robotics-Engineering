%
% range_sensor_calibration_data_collection.m
%
% collect data for a multiple calibration run
%

% initialization
clear all; close all;

% PWM value (between 0 and 255)
pwm_input = [30, 40, 50, 60, 70, 80, 90, 100, 110, 120];
pwm_input = [40,60,80,100];

% # of time steps
N=200;

% connect to the Elegoo car
robot=ElegooRobotCar('192.168.4.1');

% start watchdog for safe stop (e.g., if control-c is pressed during run)

robot.stop;
robot.setWatchdog(1000);
stopOnExit = onCleanup(@() stopRobotSafely(robot)); 

% Optional: set range sensor pan angle 
% robot.pan(95);

% Optional: set left wheel speed ratio 
a=0.95;

for k = 1:numel(pwm_input)

    disp(sprintf('iteration: %d, pwm = %g',k,pwm_input(k)));

    u = pwm_input(k);

    % *** Drive Forward and collect range sensor value at each sampling time ***

    % start timer
    for i=1:N;
        tic;
        robot.drive(u,fix(a*u));
        y_f{k}(i)=(robot.readDistance.distance);
        dt = toc;
        if i>1
            t_f{k}(i)=t_f{k}(i-1)+dt;
        else
            t_f{k}(i)=dt;
        end
    end;

    robot.stop;

    % pause before reversing
    
    % pause(.1);

    % *** Drive Backward and collect range sensor value at each sampling time ***

    u=-u;
    
    for i=1:N;
        tic;
        robot.drive(u,fix(a*u));
        y_b{k}(i)=(robot.readDistance.distance);
        dt = toc;
        if i>1
            t_b{k}(i)=t_b{k}(i-1)+dt;
        else
            t_b{k}(i)=dt;
        end
    end;

    robot.stop;

end
    
robot.close;

delete range_calibration_data.mat 
save range_calibration_data t_f t_b y_f y_b pwm_input 

function d=readRange(reading)

d=NaN;
if isstruct(reading) && isfield(reading, "distance")
    %d = double(reading.distance);
    d = reading.distance;
end

end

function stopRobotSafely(robot)
try
    robot.stop();
catch
end

end