%
% pwm2vel_cal.m
%
% calibration routine for pwm
%
%% Range Sensor Calibration UI / Script (PWM 10 to 120)
clear all; close all; clc;

% Connect to the robot
robot = ElegooRobotCar('192.168.4.1');
robot.pan(95);

% Define PWM sweep range from 10 to 120 in increments of 10
pwm_list = (30:10:120)';
N = 200;
a = 0.95;

% Initialize data table to store results: [PWM, Measured Velocity (cm/s)]
new_table = [pwm_list, zeros(length(pwm_list), 1)];

figure('Name', 'Calibration Live View', 'NumberTitle', 'off');

for idx_pwm = 1:length(pwm_list)
    uu = pwm_list(idx_pwm);
    fprintf('\n--- Running Calibration for PWM = %d ------\\n', uu);
    
    %% Forward Motion Test
    yy_forward = zeros(1, N);
    tt_forward = zeros(1, N);
    
    robot.drive(uu, fix(a * uu));
    tic;
    for i = 1:N
        yy_forward(i) = robot.readDistance.distance;
        tt_forward(i) = toc;
    end
    robot.stop();
    pause(1);
    
    %% Backward Motion Test
    yy_backward = zeros(1, N);
    tt_backward = zeros(1, N);
    
    uu_back = -uu;
    robot.drive(uu_back, fix(a * uu_back));
    tic;
    for i = 1:N
        yy_backward(i) = robot.readDistance.distance;
        tt_backward(i) = toc;
    end
    robot.stop();
    
    %% Remove Spurious Data (using provided sample logic)[span_8](start_span)[span_8](end_span)
    diffyf = [0 diff(yy_forward)];
    indf = find(abs(diffyf) > 2);
    cleanindf = setdiff(1:numel(yy_forward), indf);
    y_f = yy_forward(cleanindf)';
    t_f = tt_forward(cleanindf)';
    
    diffyb = [0 diff(yy_backward)];
    indb = find(abs(diffyb) > 2);
    cleanindb = setdiff(1:numel(yy_backward), indb);
    y_b = yy_backward(cleanindb)';
    t_b = tt_backward(cleanindb)';
    
    % Fit a line to cleaned up data[span_9](start_span)[span_9](end_span)
    n_f = length(t_f);
    n_b = length(t_b);
    
    if n_f < 5 || n_b < 5
        warning('PWM %d did not collect enough valid points. Skipping...', uu);
        continue;
    end
    
    A = [ [-t_f ones(n_f,1) zeros(n_f,1)]; [t_b zeros(n_b,1) ones(n_b,1)] ];
    xx = pinv(A) * [y_f;y_b];
    
    % Display velocity slope for current PWM[span_10](start_span)[span_10](end_span)
    v_current = xx(1);
    disp(sprintf('PWM %d -> Velocity (cm/s): %g', uu, v_current));
    
    % Store in table[span_11](start_span)[span_11](end_span)
    table_idx = find(new_table(:, 1) == abs(uu));
    if ~isempty(table_idx)
        new_table(table_idx, 2) = v_current;
    end
    
    % Plot fits for verification[span_12](start_span)[span_12](end_span)
    subplot(1, 2, 1);
    plot(t_f, y_f, '.', t_f, A(1:n_f,1:2)*xx(1:2), '-');
    title(sprintf('Forward Fit (PWM = %d)', uu));
    xlabel('Time (s)'); ylabel('Distance (cm)'); grid on;
    
    subplot(1, 2, 2);
    plot(t_b, y_b, '.', t_b, A(n_f+1:n_f+n_b,[1,3])*[xx(1);xx(3)], '-');
    title(sprintf('Backward Fit (PWM = %d)', uu));
    xlabel('Time (s)'); ylabel('Distance (cm)'); grid on;
    drawnow;
end

robot.close();

%% Filter out any unmeasured (zero) entries if tests were skipped
valid_rows = new_table(:, 2) ~= 0;
pwm = new_table(valid_rows, 1);
v = new_table(valid_rows, 2);

%% Fit Slope to Create a Mapping from Velocity (cm/s) to PWM[span_13](start_span)[span_13](end_span)
% Model: PWM = x_pwm(1) * v + x_pwm(2)
A_pwm = [v ones(size(v))];
x_pwm = pinv(A_pwm) * pwm;

%
previous_cal = ...
    [ [30;40;50;60;80;100;120] [8.44;11.71;15.41;18.44;26.00;33.95;40.38]];

figure;
%plot(v, pwm, 'o-', v, A_pwm * x_pwm, 'x-', 'LineWidth', 1.5);
plot(v, pwm, 'o-', v, A_pwm * x_pwm, 'x-', ...
    previous_cal(:,2),previous_cal(:,1),'s:','LineWidth', 1.5);
xlabel('Velocity (cm/s)');
ylabel('PWM Input');
title('Mapping from Velocity to PWM');
%legend('Measured Data', 'Linear Fit Line', 'Location', 'southeast');
legend('Measured Data', 'Linear Fit Line', 'Previous Data','Location', 'southeast');
grid on;

disp('-------------------------------------------');
disp('Final Velocity-to-PWM Mapping Equation:');
disp(sprintf('PWM = %.4f * v + %.4f', x_pwm(1), x_pwm(2)));
disp('-------------------------------------------');
