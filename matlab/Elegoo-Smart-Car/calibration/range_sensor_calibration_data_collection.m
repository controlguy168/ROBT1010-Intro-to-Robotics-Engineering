%
% range_sensor_calibration_data_collection.m
%
% collect data for a multiple calibration run
%

% initialization
clear all; close all;

% PWM value (between 0 and 255)
pwm_input = [30, 40, 50, 60, 70, 80, 90, 100, 110, 120];

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
a=1.00;

for k = 1:numel(pwm_input)

    u = pwm_input(k);

    % *** Drive Forward and collect range sensor value at each sampling time ***
    robot.drive(u,fix(a*u));

    % start timer
    tic;
    
    for i=1:N;
        y_f{k}(i)=robot.readDistance.distance;
        t_f{k}(i)=toc;
    end;

    robot.stop

    % pause before reversing
    
    pause(2);

    % *** Drive Backward and collect range sensor value at each sampling time ***

    u=-u;
    robot.drive(u,fix(a*u));

    tic;
    for i=1:N;
        y_b{k}(i)=robot.readDistance.distance;
        t_b{k}(i)=toc;
    end;

    robot.stop

end
    
robot.close;

save range_calibration_data t_f t_b y_f y_b pwm_input 

function stopRobotSafely(robot)
try
    robot.stop();
catch
end

end