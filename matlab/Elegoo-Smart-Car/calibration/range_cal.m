clear all; close all;

robot=ElegooRobotCar('192.168.4.1');

robot.pan(95);

a=1.00;
a=1.05;
uu=40;N=200;
robot.drive(uu,fix(a*uu));
tic;for i=1:N;
    yy_forward(i)=robot.readDistance.distance;
    tt_forward(i)=toc;
end;
robot.stop
pause(.5);

uu=-uu;
robot.drive(uu,fix(a*uu));
tic;for i=1:N;
    yy_backward(i)=robot.readDistance.distance;
    tt_backward(i)=toc;
end;
robot.stop

%pwm_over_v = xx(1);
pwm_v_table=[ [30;40;50;60;80;100;120] [8.44;11.85;15.41;17.84;25.51;32.86;38.67]]
pwm_over_v = interp1(pwm_v_table(:,1),pwm_v_table(:,2),abs(uu));

figure(1);plot(tt_forward,yy_forward,tt_forward,-pwm_over_v*tt_forward+yy_forward(1))
figure(2);plot(tt_backward,yy_backward,tt_backward,pwm_over_v*tt_backward+yy_backward(1))

%
% remove spurious data
%

diffyf=[0 diff(yy_forward)];
indf = find(abs(diffyf)>2);
cleanindf = setdiff(1:numel(yy_forward),indf);
y_f = yy_forward(cleanindf)';
t_f = tt_forward(cleanindf)';

diffyb=[0 diff(yy_backward)];
indb = find(abs(diffyb)>2);
cleanindb = setdiff(1:numel(yy_backward),indb);
y_b = yy_backward(cleanindb)';
t_b = tt_backward(cleanindb)';

figure(3);plot(t_f,y_f,'-',tt_forward,yy_forward,':');
figure(4);plot(t_b,y_b,'-',tt_backward,yy_backward,':');

% t_f1=input('enter start time for forward motion fit: ');
% t_f2=input('enter end time for forward motion fit: ');
% t_b1=input('enter start time for backward motion: ');
% t_b2=input('enter end time for backward motion: ');
% 
% indf1= min(find(tt_forward>t_f1));
% indf2= min(find(tt_forward>t_f2));
% indb1= min(find(tt_backward>t_b1));
% indb2= min(find(tt_backward>t_b2));

% nnf1=indf1;nnf2=indf2;
% rangef=(nnf1:nnf2);
% n_forward=length(tt_forward);
% y_f=yy_forward(rangef)';
% t_f=tt_forward(rangef)';
% 
% nnb1=indb1;nnb2=indb2;
% rangeb=(nnb1:nnb2);
% n_backward=length(tt_backward);
% y_b=yy_backward(rangeb)';
% t_b=tt_backward(rangeb)';

n_f=length(t_f);
n_b=length(t_b);

A = [ [-t_f ones(n_f,1) zeros(n_f,1)]; [t_b zeros(n_b,1) ones(n_b,1)] ];
%A_full = [ [-tt_forward' ones(n_forward,1) zeros(n_forward,1)]; ...
%    [tt_backward' zeros(n_backward,1) ones(n_backward,1)] ];
xx = pinv(A) * [y_f;y_b];

figure(11);plot(t_f,y_f,t_f,A(1:n_f,1:2)*xx(1:2))
figure(12);plot(t_b,y_b,t_b,A(n_f+1:n_f+n_b,[1,3])*[xx(1);xx(3)])

%figure(21);plot(tt_forward,yy_forward,tt_forward,A_full(1:n_forward,1:2)*xx(1:2))
%figure(22);plot(tt_backward,yy_backward,tt_backward,A_full(n_forward+1:n_forward+n_backward,[1,3])*[xx(1);xx(3)])

disp(sprintf('velocity (cm) per pwm percentage %g',xx(1)/uu))

robot.close;