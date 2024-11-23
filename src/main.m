% Vehicle Model Parameters
M = 1420;                % Vehicle mass (kg)
rho = 1.205;             % Air density (kg/m^3)
A_f = 1.7;               % Frontal area (m^2)
C_d = 0.36;              % Drag coefficient (air resistance)
f_rr = 0.011;            % Rolling resistance coefficient
eta_t = 0.94;            % Drive train total efficiency
r_w = 0.33;              % Dynamic tire radius (m)
g = 9.81;                % Gravitational acceleration (m/s^2)
alpha = 0;               % Road slope (flat road)

% Acceleration limits
a_h_max = 0.4 * g;       % Maximum allowed acceleration (m/s^2)
a_h_bmax = g;
F_b_max = M*a_h_bmax;    % Calculated from F=ma, where a = 0.4*g
T_peak = 260;            % Peak Engine Torque (NM)
w_f_peak = 315;          % Peak Engine Speed (rad/sec)
w_f_max = 733;           % Max Engine Speed (rad/sec)
v_lim = 30;
% Time settings
dt = 0.1;                % Time step (seconds)
N = 500;                 % Number of time steps
time = (0:N-1)*dt;       % Time vector
t_react = 0.8;           % Reaction Time (seconds)
% Define the state struct x
x = struct();
x.v_h = zeros(N, 1);      % Vehicle speed (m/s)
x.s_h = zeros(N, 1);      % Vehicle position (m)
x.n_g = ones(N, 1);      % Gear position (initialized to gear 0 for simplicity)

% Define the control input struct u
u = struct();
u.T_f = 200* ones(N, 1); % Engine torque (Nm)
u.F_b = zeros(N, 1);      % Brake force (N)
u.u_g = zeros(N, 1);      % Gearshift command (shift, sustain, upshift)

% Gear ratio array (I_g) for each gear (AMT with 5 gears)
I_g = [17.23, 9.78, 6.42, 4.89, 4.08];  % Gear ratios for 5 gears

%% 
w_f = @(k) min(30/(pi*r_w) * I_g(x.n_g(k)) * x.v_h(k), w_f_max);
T_f_max = @(k) T_peak * min(1, w_f_peak/w_f(k));
%% Forces and Accelerations
% Function to calculate acceleration components
get_accelerations = @(x, k) ...
    (rho * A_f * C_d / (2 * M)) * x.v_h(k)^2 + ...   % Air resistance
    g * f_rr * cos(alpha) + ...                 % Rolling resistance
    g * sin(alpha);                             % Gravity (set to 0 for flat road)
% Updated Vehicle dynamics with gearshift, forces, and constraints
f = @(x, u, k) [ ...
    (eta_t / (M * r_w)) * u.T_f(k) * I_g(x.n_g(k)) - (u.F_b(k) / M) - get_accelerations(x, k) % Speed update with constraints (f1)
    x.v_h(k);                                                               % Position update (f2)
    max(1, min(x.n_g(k) + u.u_g(k), length(I_g)))];                          % Gear position update with bounds (f3)

% Initial conditions
x.v_h(1) = 0;              % Initial speed (m/s)
x.s_h(1) = 0;              % Initial position (m)
x.n_g(1) = 1;              % Initial gear (starting in gear 1)

%% Velocity constraints
%v_h_m1
%v_h_m2 = -a_h_bmax * t_react
T_w_max = @(k) T_peak * I_g(x.n_g(k)) * eta_t;
v_h_m3 = @(k) x.v_h(k) + eta_t * T_w_max(k) - get_accelerations(x, k); % (13c)


%% Fuel Rate Polynomial
kappa = [0.1, 0.01, 0.001;    
         0.05, 0.005, 0.0005; 
         0.01, 0.001, 0.0001];
TWF = @(k) [T_f(k)^0 * w_f(k)^0, T_f(k)^0 * w_f(k)^1, T_f(k)^0 * w_f(k)^2;
            T_f(k)^1 * w_f(k)^0, T_f(k)^1 * w_f(k)^1, T_f(k)^1 * w_f(k)^2;
            T_f(k)^2 * w_f(k)^0, T_f(k)^2 * w_f(k)^1, T_f(k)^2 * w_f(k)^2];
m_fuel_dot = @(k) kappa .* TWF(k); % (7)
%% Penalties
w_r = 1;
w_c = 0.5;
v_ref = 30;
v_f = 30; % terminal speed needs to be clarified
phi = 1;
P = @(x, u, k) (u.T_f(k) * I_g(x.n_g(k)) - u.T_f(k - 1) * I_g(x.n_g(k - 1)))^2 + u.F_b(k)^2; % (6)
L = @(x, u, k) m_fuel_dot(k) + w_r*(x.v_h(k) - v_ref)^2 + w_c*P(x, u, k); % (5)
J = @(x, u, k) L(x, u, k)*dt + phi*(x.v_h(k + 1) - v_f)^2;  % (4)
%% Simulation Loop with Constraints

for k = 2:N-1
    % Apply physical constraints to the control inputs
    u.T_f(k) = min(max(u.T_f(k), 0), T_f_max(k));   % Constrain engine torque based on wf
    % Compute the current acceleration based on all forces
    f_update = f(x, u, k);                       % Get the full dynamics output
    total_acceleration = f_update(1);           % Extract the total acceleration (first element)
    
    % Apply maximum acceleration limit
    if total_acceleration > a_h_max
        % Scale down the engine torque to enforce acceleration limit
        scale_factor = a_h_max / total_acceleration;
        u.T_f(k) = u.T_f(k) * scale_factor;  % Scale down the engine torque
        
        % Recalculate the total acceleration with the scaled torque
        f_update = f(x, u, k);                    
        total_acceleration = f_update(1);  % Updated acceleration after scaling
    end
    
    % Update the states using the limited acceleration
    if k > 2
        x.v_h(k+1) = min(min(x.v_h(k) + total_acceleration * dt, v_lim) , v_h_m3(k));
    else
        x.v_h(k+1) = min(x.v_h(k) + total_acceleration * dt, v_lim); % Speed update
    end
    x.s_h(k+1) = x.s_h(k) + x.v_h(k) * dt;              % Position update
    x.n_g(k+1) = f_update(3);                           % Gear position update
    
    u.F_b(k + 1) = u.F_b(k);
    u.T_f(k + 1) = u.T_f(k);
    u.u_g(k + 1) = u.u_g(k);

    %%%%
end

% Plot Results
figure;
subplot(4, 1, 1);
plot(time, x.v_h, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Vehicle Speed over Time');
grid on;

subplot(4, 1, 2);
plot(time, x.s_h, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position (m)');
title('Vehicle Position over Time');
grid on;

subplot(4, 1, 3);
plot(time, x.n_g, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Gear Position');
title('Gear Position over Time');
grid on;

subplot(4, 1, 4);
plot(time(2:end), u.T_f(2:end), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Engine Torque');
title('Engine Torque over Time');
grid on;

%% Preceeding Vehicle States
px = struct();
px.v_p = zeros(N, 1);      % Vehicle speed (m/s)
px.s_p = zeros(N, 1);      % Vehicle position (m)
%px.s_p(k + 1) = px.s_p(k) + px.x_p(k)*dt % distance update 
%% Prediction


%% p-Functions