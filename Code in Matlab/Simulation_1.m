clc
clear

%% Simulation detail
stop_time = 6000;
sampling_interval = 1e-4;
num_data = int32(stop_time / sampling_interval) + 1;
t = 0:sampling_interval:stop_time;

%% System parameters
% System's parameters
K = zeros(3,1) + 1000;   % temporary effect: the convergence of error when t coming to infinity
k4 = 0.01;               % k4 > 0; k4 directly affect the tracking performance
K_4 = k4 * eye(2);
% Robot's parameters
I = 1;
r = 1;
R = 1;
d = 1;
m = 1.5;

%% Variable declaration
x = zeros(1, num_data);
y = zeros(1, num_data);
theta = zeros(1, num_data);
v = zeros(2, num_data);
tau_l = zeros(1, num_data);
tau_r = zeros(1, num_data);

e_x = zeros(1, num_data);
e_y = zeros(1, num_data);
e_theta = zeros(1, num_data);

% Desired trajectory
x_r = zeros(1, num_data);
y_r = zeros(1, num_data);
theta_r = zeros(1, num_data);
v_r = zeros(1, num_data) + 1.5;
w_r = zeros(1, num_data) + 1.2;

derivative_x_r = zeros(1, num_data);
derivative_y_r = zeros(1, num_data);
derivative_theta_r = zeros(1, num_data);

x_r(1) = 0;
y_r(1) = 0;
theta_r(1) = 0;

derivative_x_r(1) = v_r(1) * cos(theta_r(1));
derivative_y_r(1) = v_r(1) * sin(theta_r(1));
derivative_theta_r(1) = w_r(1);

x_r(2) = derivative_x_r(1) * sampling_interval + x_r(1);
y_r(2) = derivative_y_r(1) * sampling_interval + y_r(1);
theta_r(2) = derivative_theta_r(1) * sampling_interval + theta_r(1);

for i = 2 : (num_data)
    derivative_x_r(i) = v_r(i) * cos(theta_r(i));
    derivative_y_r(i) = v_r(i) * sin(theta_r(i));
    derivative_theta_r(i) = w_r(i);
    
    if i == num_data
        break
    end
    
    x_r(i + 1) = derivative_x_r(i) * sampling_interval + x_r(i);
    y_r(i + 1) = derivative_y_r(i) * sampling_interval + y_r(i);
    theta_r(i + 1) = derivative_theta_r(i) * sampling_interval + theta_r(i);
end

%% Simulation
% Initital value
x(1) = 0;
y(1) = 0;
theta(1) = 0;
v(1, 1) = 0;
v(2, 1) = 0;

v_c = zeros(2, num_data);

e_x(1) = cos(theta(1)) * (x_r(1) - x(1)) + sin(theta(1)) * (y_r(1) - y(1));
e_y(1) = (-1) * sin(theta(1)) * (x_r(1) - x(1)) + cos(theta(1)) * (y_r(1) - y(1));
e_theta(1) = theta_r(1) - theta(1);

v_c(1, 1) = v_r(1) * cos(e_theta(1)) + K(1) * e_x(1);
v_c(2, 1) = w_r(1) + K(2) * v_r(1) * e_y(1) + K(3) * v_r(1) * sin(e_theta(1));

derivative_v_c = zeros(2, 1);

derivative_v_c(1, 1) = 0;
derivative_v_c(2, 1) = 0;

u = derivative_v_c + K_4 * (v_c(:, 1) - v(:, 1));

% Second state
v(:, 2) = v(:, 1);

derivative_x = cos(theta(1)) * v(1, 1);
derivative_y = sin(theta(1)) * v(1, 1);
derivative_theta = v(2, 1);

x(2) = x(1) + sampling_interval * derivative_x;
y(2) = y(1) + sampling_interval * derivative_y;
theta(2) = theta(1) + sampling_interval * derivative_theta;
%{
S = [cos(theta(1)), - d * sin(theta(1)); sin(theta(1)), d * cos(theta(1)); 0, 1];
M = [m, 0, m * d * sin(theta(1)); 0, m, - m * d * cos(theta(1)); m * d * sin(theta(1)), - m * d * cos(theta(1)), I];
V = [m * d * (derivative_theta)^2 * cos(theta(1)); m * d * (derivative_theta)^2 * sin(theta(1)); 0];
derivative_S = [- sin(theta(1)) * derivative_theta, - d * cos(theta(1)) * derivative_theta; cos(theta(1)) * derivative_theta, - d * sin(theta(1)) * derivative_theta; 0, 0];
B = (1 / r) * [cos(theta(1)), cos(theta(1)); sin(theta(1)), sin(theta(1)); R, -R];

tau = (S' * B) \ S' * M * S * u + (S' * B) \ S' * (M * derivative_S + V * S) * v(:, 1);
tau_l(2) = tau(1);
tau_r(2) = tau(2);
%}
% Third state
v(1, 3) = v(1, 2) + u(1) * sampling_interval;
v(2, 3) = v(2, 2) + u(2) * sampling_interval;

derivative_x = cos(theta(2)) * v(1, 2);
derivative_y = sin(theta(2)) * v(1, 2);
derivative_theta = v(2, 2);

x(3) = x(2) + sampling_interval * derivative_x;
y(3) = y(2) + sampling_interval * derivative_y;
theta(3) = theta(2) + sampling_interval * derivative_theta;

for i = 2 : num_data
    
    e_x(i) = cos(theta(i)) * (x_r(i) - x(i)) + sin(theta(i)) * (y_r(i) - y(i));
    e_y(i) = (-1) * sin(theta(i)) * (x_r(i) - x(i)) + cos(theta(i)) * (y_r(i) - y(i));
    e_theta(i) = theta_r(i) - theta(i);
    
    v_c(1, i) = v_r(i) * cos(e_theta(i)) + K(1) * e_x(i);
    v_c(2, i) = w_r(i) + K(2) * v_r(i) * e_y(i) + K(3) * v_r(i) * sin(e_theta(i));
    
    derivative_v_c(1, 1) = (v_c(1, i) - v_c(1, i - 1)) / sampling_interval;
    derivative_v_c(2, 1) = (v_c(2, i) - v_c(2, i - 1)) / sampling_interval;
    
    u = derivative_v_c + K_4 * (v_c(:, 2) - v(:, 2));
    %{
    if i <= num_data - 1
    % Torque i + 1st
    derivative_theta = v(2, i);
    
    S = [cos(theta(i)), - d * sin(theta(i)); sin(theta(i)), d * cos(theta(i)); 0, 1];
    M = [m, 0, m * d * sin(theta(i)); 0, m, - m * d * cos(theta(i)); m * d * sin(theta(i)), - m * d * cos(theta(i)), I];
    V = [m * d * (derivative_theta)^2 * cos(theta(i)); m * d * (derivative_theta)^2 * sin(theta(i)); 0];
    derivative_S = [- sin(theta(i)) * derivative_theta, - d * cos(theta(i)) * derivative_theta; cos(theta(i)) * derivative_theta, - d * sin(theta(i)) * derivative_theta; 0, 0];
    B = (1 / r) * [cos(theta(i)), cos(theta(i)); sin(theta(i)), sin(theta(i)); R, -R];

    tau = (S' * B) \ S' * M * S * u + (S' * B) \ S' * (M * derivative_S + V * S) * v(:, i);
    tau_l(i + 1) = tau(1);
    tau_r(i + 1) = tau(2);
    end
    %}
    if i <= num_data - 2
    % State i + 2nd
    v(1, i + 2) = v(1, i + 1) + u(1) * sampling_interval;
    v(2, i + 2) = v(2, i + 1) + u(2) * sampling_interval;
    
    derivative_x = cos(theta(i + 1)) * v(1, i + 1);
    derivative_y = sin(theta(i + 1)) * v(1, i + 1);
    derivative_theta = v(2, i + 1);

    x(i + 2) = x(i + 1) + sampling_interval * derivative_x;
    y(i + 2) = y(i + 1) + sampling_interval * derivative_y;
    theta(i + 2) = theta(i + 1) + sampling_interval * derivative_theta;
    end
end

























