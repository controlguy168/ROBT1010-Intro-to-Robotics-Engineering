%
% collect sensor information and find its spectrum
%

% initialization

clear all; close all;

robot=ElegooRobotCar('192.168.4.1');

% start watchdog

robot.stop;
robot.setWatchdog(1000);
stopOnExit = onCleanup(@() stopRobotSafely(robot)); 

% choose sensor

x_mean = -95;

% set speed 

w = 80;

% select input 

w_sel = input('Enter 1 for square wave, 2 for sinusoid: ');

if w_sel == 1
    k_switch = 2.5;
    dir = 1;
else
    sine_freq = .5;
end

% sampling period
t_s = 0.1;

% # of samples
N = 200;

% initialize time vector and start loop

time(1)=0;

for k = 1: N

    timer = tic;

    if w_sel == 1; 
        if mod(k,k_switch)==0;dir=-dir;end; 
    else
        w_sine = w*sin(2*pi*sine_freq*time(k));
    end
   
    while toc(timer)<t_s
         x(k)=robot.readIMU.gyro(3)-x_mean;      
         
         if w_sel== 1
            u(k) = dir * w;
         else
            u(k) = w_sine;
         end

         robot.drive(fix(u(k)),fix(-u(k)));

        dt(k) = toc(timer); % elapsed time
    end

    time(k+1)=time(k)+dt(k);
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

% Sampling period 

figure,plot(t,dt,'x',t,t_s*ones(1,N),'r:','LineWidth',1.5);grid
axis([0 t(end) 0 2*t_s]);
xlabel('time (sec)'); ylabel('Recorded sampling period (sec)');
title('Sampling Period')

% Time domain signal and input

figure,plot(t,x-mean(x),'o-','LineWidth',1.5);grid
xlabel('time (sec)');ylabel('gyro output')
title('gyro output with mean removed');

figure,
yyaxis left,plot(t,x-mean(x),'linewidth',1.5);
ylabel('gyro output');
yyaxis right,plot(t,u,'linewidth',1.5)
ylabel('command input');
xlabel('time (sec)');
title('gyro output with mean removed and command input');

% Frequency domain signal 

xf=fft(x-mean(x))/N;
f_s = 1/(N*t_s);
fvec= (0:fix(N/2))*f_s;

figure,plot(fvec,abs(xf(1:numel(fvec))),'o-','LineWidth',1.5);grid
xlabel('frequency (Hz)');ylabel('sensor output spectrum magnitude');
title('spectrum of sensor output with mean removed');

% safe exit routine
function stopRobotSafely(robot)
try
    robot.stop();
catch
end

end