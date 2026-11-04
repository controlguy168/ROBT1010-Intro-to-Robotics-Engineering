%
% remove spurious data
%

diffyf=[0 diff(yy_forward)];
indf = find(abs(diffyf)>2);
%indf=[indf(1) 20 21 22 23 indf(2:end)]
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

% fit a line to cleaned up data

n_f=length(t_f);
n_b=length(t_b);

A = [ [-t_f ones(n_f,1) zeros(n_f,1)]; [t_b zeros(n_b,1) ones(n_b,1)] ];
xx = pinv(A) * [y_f;y_b];

figure(11);plot(t_f,y_f,t_f,A(1:n_f,1:2)*xx(1:2))
figure(12);plot(t_b,y_b,t_b,A(n_f+1:n_f+n_b,[1,3])*[xx(1);xx(3)])

disp(sprintf('velocity (cm) per pwm percentage %g',xx(1)))

pwm_v_table=[ [30;40;50;60;80;100;120] [8.44;11.85;15.41;17.84;25.51;32.86;38.67]];

idx=find(pwm_v_table(:,1)==abs(uu));

new_table(idx,2) = xx(1);

%[ [30;40;50;60;80;100;120] [xxx;11.71;xxx;18.44;26.00;33.95;40.38]];

pwm=new_table(:,1);v=new_table(:,2);
A_pwm = [pwm ones(size(pwm))];
x_pwm = pinv(A_pwm) * v;

figure,plot(pwm,v,'o-',pwm,A_pwm * x_pwm,'x-')
%
% prediction of v:  v = x_pwm(1) * pwm + x_pwm (2) 
%
% range sensor prediction: x_{k+1} = x_k - v * dt 