% MATLAB script to visualize the effect of P, I, D, and PID controllers
% with varying tau and plot results including steady-state error sensitivity

% Clear workspace and command window
clear; clc; close all;

% Define system parameters
s = tf('s'); % Define the Laplace variable 's'
G = 1/(s+1); % Simple first-order system G(s) = 1 / (s + 1)

% Controller gains
Kp = 2;  % Proportional gain
Ki = 1;  % Integral gain
Kd = 0.5;  % Derivative gain

% Define different tau values
tau_values = [0.001, 0.01, 0.1, 1, 10];

% Simulation time
t = 0:0.01:5; % Time vector

% Create arrays to store steady-state error for each tau
steady_state_error = zeros(1, length(tau_values));

% Create a figure with subplots
figure;

% First subplot: Step Responses
subplot(2, 1, 1);
hold on;

% Loop through each value of tau
for i = 1:length(tau_values)
    tau = tau_values(i);  % Set current tau value
    
    % Define PID controller with current tau
    PID_controller = Kp + Ki/(tau*s) + Kd*s*tau;  % PID control
    
    % Closed-loop system for PID control with current tau
    sys_PID = feedback(PID_controller * G, 1);
    
    % Step response for the system
    [y_PID, t_PID] = step(sys_PID, t);
    
    % Calculate steady-state error (final value error from 1)
    steady_state_error(i) = abs(1 - y_PID(end));
    
    % Plot the result for current tau
    plot(t_PID, y_PID, 'LineWidth', 1.5, 'DisplayName', ['\tau = ', num2str(tau)]);
    
end

% Customize the first subplot
grid on;
xlabel('Time (seconds)');
ylabel('Amplitude');
title('Effect of \tau on PID Control - Step Responses');
legend('show', 'Location', 'Best');
hold off;

% Second subplot: Steady-State Error Sensitivity
subplot(2, 1, 2);

% Remove non-positive steady-state error values to prevent log scale issues
valid_indices = steady_state_error > 0;  % Logical index for valid errors
loglog(tau_values(valid_indices), steady_state_error(valid_indices), 'ro-', 'LineWidth', 2, 'MarkerSize', 8); % Log-log scale

% Customize the second subplot
grid on;
xlabel('\tau');
ylabel('Steady-State Error (e_t)');
title('Sensitivity of Steady-State Error to \tau');

% Adjust subplot layout
set(gcf, 'Position', [100, 100, 700, 600]); % Set figure size
