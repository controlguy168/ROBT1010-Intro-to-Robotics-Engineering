%
% distance_control.m
%
% use range sensor to control distance to wall to a specified value
%

% initialization

clear all; close all;

robot=ElegooRobotCar('192.168.4.1');

% start watchdog

robot.stop;
robot.setWatchdog(1000);
stopOnExit = onCleanup(@() stopRobotSafely(robot)); 

% set constants

xstar = 100; % target distance (cm)
k_max = 50; % maximum # of iterations
k_p = 5; % proportional feedback gain
t_s = 0.2; % assumed sampling period
%x_threshold = 2; % detection region of spurious reading
% control loop 

for k = 1: k_max    
  
    x(k)=double(robot.readDistance.distance);
    
    u(k) = k_p * ( x(k) - xstar ) * t_s;
    pwm(k) = v2pwm(u(k));

    timer = tic;
    while toc(timer)<t_s
        robot.drive(pwm(k),pwm(k));
        dt(k) = toc(timer); % elapsed time
    end
    
    robot.stop;

    disp(sprintf('k= %d, x=%g, u=%g, PWM=%d, dt =%g',k,x(k),u(k),pwm(k), dt(k)));
end

robot.close;

function pwm=v2pwm(v)

  pwm = max(min(fix(3.0207 * v - 1.73),255),-255);
  
end

function stopRobotSafely(robot)
try
    robot.stop();
catch
end

end