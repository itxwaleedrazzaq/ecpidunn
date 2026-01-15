% MATLAB script to visualize the effect of P, I, D, and PID controllers
% with annotations for steady-state error, rise-time, settling-time, etc.

% Clear workspace and command window
clear; clc; close all;

% Define system parameters
s = tf('s'); % Define the Laplace variable 's'
G = 1/(s+1); % Simple first-order system G(s) = 1 / (s + 1)

% Controller gains
Kp = 2;  % Proportional gain
Ki = 1;  % Integral gain
Kd = 0.5;  % Derivative gain

% Define controllers
P_controller = Kp;                    % Proportional control
I_controller = Ki/s;                   % Integral control
D_controller = Kd*s;                   % Derivative control
PID_controller = Kp + Ki/s + Kd*s;     % PID control

% Closed-loop systems for each controller
sys_simple = feedback(G, 1);            % No controller
sys_P = feedback(P_controller * G, 1);  % Proportional
sys_I = feedback(I_controller * G, 1);  % Integral
sys_D = feedback(D_controller * G, 1);  % Derivative
sys_PID = feedback(PID_controller * G, 1); % PID

% Simulation time
t = 0:0.01:30; % Time vector

% Step responses for each system
[y_simple, t_simple] = step(sys_simple, t);
[y_P, t_P] = step(sys_P, t);
[y_I, t_I] = step(sys_I, t);
[y_D, t_D] = step(sys_D, t);
[y_PID, t_PID] = step(sys_PID, t);

% Calculate system characteristics for each controller
info_simple = stepinfo(sys_simple, 'RiseTimeLimits', [0.1, 0.9]);
info_P = stepinfo(sys_P, 'RiseTimeLimits', [0.1, 0.9]);
info_I = stepinfo(sys_I, 'RiseTimeLimits', [0.1, 0.9]);
info_D = stepinfo(sys_D, 'RiseTimeLimits', [0.1, 0.9]);
info_PID = stepinfo(sys_PID, 'RiseTimeLimits', [0.1, 0.9]);

% Plot the results
figure('Position', [100, 100, 800, 600]);
hold on;

% Plot time responses for each system
plot(t_simple, y_simple, 'm', 'LineWidth', 2); % Simple system
plot(t_P, y_P, 'r', 'LineWidth', 2);           % Proportional
plot(t_I, y_I, 'g', 'LineWidth', 2);           % Integral
plot(t_D, y_D, 'b', 'LineWidth', 2);           % Derivative
plot(t_PID, y_PID, 'k', 'LineWidth', 2);       % PID

% Add grid, labels, title
grid on;
xlabel('Time (seconds)', 'FontSize', 14);
ylabel('Amplitude', 'FontSize', 14);
title('Time Response of Different Controllers', 'FontSize', 16);
legend('No Control', 'Proportional (P)', 'Integral (I)', 'Derivative (D)', 'PID', 'Location', 'Best', 'FontSize', 12);

% Plot Characteristics for Each System with different orientations

% Simple System
ss_error_simple = 1 - y_simple(end);
rise_time_simple = info_simple.RiseTime;
settling_time_simple = info_simple.SettlingTime;
text(23, 0.8, sprintf('Simple:\nRise Time: %.2f s\nSettling Time: %.2f s\nSS Error: %.2f', rise_time_simple, settling_time_simple, ss_error_simple), 'Color', 'm', 'FontSize', 12);

% Proportional Controller (P)
ss_error_P = 1 - y_P(end);
rise_time_P = info_P.RiseTime;
settling_time_P = info_P.SettlingTime;
text(15, 0.8, sprintf('P:\nRise Time: %.2f s\nSettling Time: %.2f s\nSS Error: %.2f', rise_time_P, settling_time_P, ss_error_P), 'Color', 'r', 'FontSize', 12);

% Integral Controller (I)
ss_error_I = 1 - y_I(end);
rise_time_I = info_I.RiseTime;
settling_time_I = info_I.SettlingTime;
text(8, 0.4, sprintf('I:\nRise Time: %.2f s\nSettling Time: %.2f s\nSS Error: %.2f', rise_time_I, settling_time_I, ss_error_I), 'Color', 'g', 'FontSize', 12);

% Derivative Controller (D)
ss_error_D = 1 - y_D(end);
rise_time_D = info_D.RiseTime;
settling_time_D = info_D.SettlingTime;
text(8, 0.8, sprintf('D:\nRise Time: %.2f s\nSettling Time: %.2f s\nSS Error: %.2f', rise_time_D, settling_time_D, ss_error_D), 'Color', 'b', 'FontSize', 12);

% PID Controller
ss_error_PID = 1 - y_PID(end);
rise_time_PID = info_PID.RiseTime;
settling_time_PID = info_PID.SettlingTime;
text(8, 1.1, sprintf('PID:\nRise Time: %.2f s\nSettling Time: %.2f s\nSS Error: %.2f', rise_time_PID, settling_time_PID, ss_error_PID), 'Color', 'k', 'FontSize', 12);

% Set tick parameters for professional look
set(gca, 'FontSize', 12);
set(gca, 'Box', 'off');

hold off;
