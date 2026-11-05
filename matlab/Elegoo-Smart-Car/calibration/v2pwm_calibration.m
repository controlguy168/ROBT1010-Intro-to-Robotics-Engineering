%
% v2pwm_calibration:
%
% This program will fit the collected calibration data for different
% constant pwm_input values and the output range data in forward and
% backward motion to a line relating drive velocity to pwm values
%

clear all;close all;

load range_calibration_data

%
% linear fit to extract v
%

for k = 1:numel(pwm_input)

    %
    % Range sensor via wifi connection sometimes return spurious data.
    % The following section tries to remove spurious data
    %

    % for forward motion
    p_f = polyfit(t_f{k},y_f{k},1);
    y_f_fit = p_f(1) * t_f{k} + p_f(2);
    std_f_err = std( y_f{k} - y_f_fit );
    idx=find(abs( y_f{k} - y_f_fit ) > std_f_err);
    cleanindf = setdiff(1:numel(y_f{k}),idx);
    y_f_clean = y_f{k}(cleanindf)';
    t_f_clean = t_f{k}(cleanindf)';
    figure;plot(t_f{k},y_f{k},'x-',t_f_clean,y_f_clean,'o','linewidth',1.5);
    xlabel('time (sec)');ylabel('range sensor distance (cm)');legend('measured','cleaned');
    title(sprintf('PWM Input = %d',pwm_input(k)))

    % for backward motion
    p_b = polyfit(t_b{k},y_b{k},1);
    y_b_fit = p_b(1) * t_b{k} + p_b(2);
    std_b_err = std( y_b{k} - y_b_fit );
    idx=find(abs(y_b{k}-y_b_fit) > std_b_err);
    cleanindf = setdiff(1:numel(y_b{k}),idx);
    y_b_clean = y_b{k}(cleanindf)';
    t_b_clean = t_b{k}(cleanindf)';
    figure;plot(t_b{k},y_b{k},'x-',t_b_clean,y_b_clean,'o','linewidth',1.5);
    xlabel('time (sec)');ylabel('range sensor distance (cm)');legend('measured','cleaned');
    title(sprintf('PWM Input = %d',pwm_input(k)))

    %
    % linear fit again with cleaned data: p(1) * t + p(2) for y (forward and backward)
    %
    p_f = polyfit(t_f_clean,y_f_clean,1);
    p_b = polyfit(t_b_clean,y_b_clean,1);

    figure;plot(t_f{k},y_f{k},'.',t_f{k},p_f(1)*t_f{k}+p_f(2),'-','linewidth',1.5);
    xlabel('time (sec)');ylabel('range sensor distance (cm)');legend('cleaned','fit');
    title(sprintf('PWM Input = %d',pwm_input(k)))
    figure;plot(t_b{k},y_b{k},'.',t_b{k},p_b(1)*t_b{k}+p_b(2),'-','linewidth',1.5);
    xlabel('time (sec)');ylabel('range sensor distance (cm)');legend('cleaned','fit');
    title(sprintf('PWM Input = %d',pwm_input(k)))

    v_f(k)=-p_f(1);
    v_b(k)=-p_b(1);

    disp(sprintf('forward velocity (cm/s) for pwm %f = %f',pwm_input(k),p_f(1)))
    disp(sprintf('backward velocity (cm/s) for pwm %f = %f',-pwm_input(k),p_b(1)))

end

%
% plot pwm vs. forward velocity 
% 

figure;
plot(v_f,pwm_input,'o-',v_b,-pwm_input,'s-','LineWidth',1.5);
xlabel('velocity (cm/s)'); ylabel('PWM'); 
legend('forward','backward','Location','best'); grid on;

%
% fit to a line   PWM = p(1)* v + p(2)
%
p = polyfit([v_f v_b],[pwm_input,-pwm_input],1);
figure;
plot(v_f,pwm_input,'bo-',v_b,-pwm_input,'bo-',...
    [v_f v_b], p(1)*[v_f v_b]+p(2),'r--','LineWidth',1.5);
xlabel('velocity (cm/s)'); ylabel('PWM'); 
legend('forward','backward','Location','best'); grid on;

fprintf('PWM = %.4f * Velocity + %.4f\n',p(1),p(2));
