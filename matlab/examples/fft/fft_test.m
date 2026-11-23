%
% collect sensor information and find its spectrum
%

% initialization

clear all; close all;

%robot=ElegooRobotCar('192.168.4.1');
robot=SimulatedElegooRobotCar();

% start watchdog

robot.stop;
robot.setWatchdog(1000);
stopOnExit = onCleanup(@() stopRobotSafely(robot)); 

% choose sensor

sensor_sel = input('enter 1 for w_z, 2 for a_y, 3 for distance: ');
switch sensor_sel 
    case 2 
        sensor_label='Acceleration in body y direction';
    case 3
        sensor_label='Range Sensor';            
    otherwise
        sensor_label='Angular velocity about z axis';
end

% sampling period
t_s = 0.1; % 100 ms

% # of samples
N = 200;

% 
for k = 1: N

    timer = tic;
    while toc(timer)<t_s
        switch sensor_sel 
            case 2 
                x(k)=robot.readIMU.accel(2);
            case 3
                x(k)=robot.readDistance.distance;                
            otherwise
                x(k)=robot.readIMU.gyro(3);
        end
        dt(k) = toc(timer); % elapsed time
    end

end

robot.stop;

%
% spectral analysis
%

t=(0:N-1)*t_s;

% statistics

x_mean=mean(x);
x_std = std(x);
x_max = max(abs(x-x_mean));
fprintf('\n mean=%g, stdev=%g, maxdev=%g \n\n',x_mean,x_std,x_max);

figure,plot(t,dt,'x',t,t_s*ones(1,N),'r:','LineWidth',1.5);grid
axis([0 t(end) 0 2*t_s]);
xlabel('time (sec)'); ylabel('Recorded sampling period (sec)');
title(['Sampling Period: ',sensor_label])

figure,plot(t,x-mean(x),'o-','LineWidth',1.5);grid
xlabel('time (sec)');ylabel('sensor output')
title(['sensor output with mean removed: ',sensor_label]);

xf=fft(x-mean(x))/N;
f_s = 1/(N*t_s);
fvec= (0:fix(N/2))*f_s;

figure,plot(fvec,abs(xf(1:numel(fvec))),'o-','LineWidth',1.5);grid
xlabel('frequency (Hz)');ylabel('sensor output spectrum magnitude');
title(['spectrum of sensor output with mean removed: ',sensor_label]);

% safe exit routine
function stopRobotSafely(robot)
try
    robot.stop();
catch
end

end