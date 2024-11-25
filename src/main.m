% Vehicle Model Parameters
clearvars; clc;
global M rho A_f C_d f_rr eta_t r_w g alpha a_h_max a_h_bmax F_b_max T_peak w_f_peak w_f_max v_lim I_g v_h_max dt;
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
x_ = struct();
x_.v_h = zeros(N, 1);      
x_.s_h = zeros(N, 1);      
x_.n_g = ones(N, 1);
u_ = struct();
u_.T_f = zeros(N, 1); 
u_.F_b = zeros(N, 1);      
u_.u_g = zeros(N, 1);     
lambda = zeros(N, 1);

v_h_max = 30*ones(N + 1,1);
% Preceeding Vehicle States
px = struct();
px.v_p = zeros(N, 1);      % Vehicle speed (m/s)
px.s_p = zeros(N, 1);      % Vehicle position (m)


% Define the control input struct u
u = struct();
u.T_f = 200* ones(N, 1); % Engine torque (Nm)
u.F_b = zeros(N, 1);      % Brake force (N)
u.u_g = zeros(N, 1);      % Gearshift command (shift, sustain, upshift)

% Gear ratio array (I_g) for each gear (AMT with 5 gears)
I_g = [17.23, 9.78, 6.42, 4.89, 4.08];  % Gear ratios for 5 gears
%%


%% W_f and T_f_max

%w_f = @(x,k) min(30/(pi*r_w) * I_g(x.n_g(k)) * x.v_h(k), w_f_max);

%T_f_max = @(x,k) T_peak * min(1, w_f_peak/w_f(x,k));
%% Forces and Accelerations
% Function to calculate acceleration components
% get_accelerations = @(x, k) ...
%     (rho * A_f * C_d / (2 * M)) * x.v_h(k)^2 + ...   % Air resistance
%     g * f_rr * cos(alpha) + ...                 % Rolling resistance
%     g * sin(alpha);                             % Gravity (set to 0 for flat road)
% Updated Vehicle dynamics with gearshift, forces, and constraints
% f = @(x, u, k) [ ...
%     (eta_t / (M * r_w)) * u.T_f(k) * I_g(x.n_g(k)) - (u.F_b(k) / M) - get_accelerations(x, k) % Speed update with constraints (f1)
%     x.v_h(k);                                                               % Position update (f2)
%     max(1, min(x.n_g(k) + u.u_g(k), length(I_g)))];                          % Gear position update with bounds (f3)

% Initial conditions
x.v_h(1) = 0;              % Initial speed (m/s)
x.s_h(1) = 0;              % Initial position (m)
x.n_g(1) = 1;              % Initial gear (starting in gear 1)

px.v_p(1) = 20;
px.s_p(1) = 100;
px.v_p(2) = 20;
px.s_p(2) = 100;

%% Velocity constraints
%v_h_m1
%v_h_m2 = -a_h_bmax * t_react
%T_w_max = @(x,k) T_peak * I_g(x.n_g(k)) * eta_t;
%v_h_m3 = @(x,k) x.v_h(k) + eta_t * T_w_max(x,k) - get_accelerations(x, k); % (13c)


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



%% Corrected p coefficient 
% % p1: coefficient of T_f^2 term (κ2,j terms)
% p1 = @(x,k) kappa(3,1) + kappa(3,2)*w_f(k) + kappa(3,3)*w_f(k)^2 + w_c * I_g(x.n_g(k))^2;
% 
% % p2: coefficient of T_f term (κ1,j terms)
% p2 = @(x,k) (kappa(2,1) + kappa(2,2)*w_f(k) + kappa(2,3)*w_f(k)^2) + ...
%           lambda(k) * eta_t/(M*r_w) * I_g(x.n_g(k)) - ...
%           2*w_c * I_g(x.n_g(k)) * I_g(x.n_g(k-1)) * u.T_f(k-1);
% 
% % p3 and p4 remain the same
% p3 = @(x,k) w_c;
% p4 = @(x,k) -lambda(k)/M;
% 
% % p5: constant term (κ0,j terms)
% p5 = @(x,lambda,k) (kappa(1,1) + kappa(1,2)*w_f(k) + kappa(1,3)*w_f(k)^2) + ...
%           w_r * (x.v_h(k) - v_ref)^2 - ...
%           lambda(k) * a_k(k) + ...
%           w_c * (u.T_f(k-1) * I_g(x.n_g(k-1)))^2;
%%
% Equations 23-30 as anonymous functions

% Basic Hamiltonian (23)
%H = @(x,u,k) p1(x,k)*u.T_f(k)^2 + p2(x,k)*u.T_f(k) + p3(x,k)*u.F_b(x,k)^2 + p4(x,k)*u.F_b(k) + p5(x,k);

% Drive and brake mode Hamiltonians (24)
H_drive = @(x,u,k) p1(x,k)*u.T_f(k)^2 + p2(x,k)*u.T_f(k) + p5(x,k);
H_brake = @(x,u,k) p3(x,k)*u.F_b(x,k)^2 + p4(x,k)*u.F_b(k) + p5(x,k);

% Maximum torque constraint (26)
% T_max = @(x,k) min(T_f_max(x,k), ...
%     ((v_h_max(k+1) - x.v_h(k))/dt + get_accelerations(x,k)) * M*r_w/eta_t);

%% Simulation Loop with Constraints

for k = 2:N-1
    % Apply physical constraints to the control inputs
    u.T_f(k) = min(max(u.T_f(k), 0), T_f_max(x, k));   % Constrain engine torque based on wf
    % Compute the current acceleration based on all forces
    f_update = vehicle_dynamics(x, u, k);                       % Get the full dynamics output
    total_acceleration = f_update(1);           % Extract the total acceleration (first element)
    
    % Apply maximum acceleration limit
    if total_acceleration > a_h_max
        % Scale down the engine torque to enforce acceleration limit
        scale_factor = a_h_max / total_acceleration;
        u.T_f(k) = u.T_f(k) * scale_factor;  % Scale down the engine torque
        
        % Recalculate the total acceleration with the scaled torque
        f_update = vehicle_dynamics(x, u, k);                    
        total_acceleration = f_update(1);  % Updated acceleration after scaling
    end
    % Update the states using the limited acceleration
    % PMP STUFF
    %lambda(k) = find_optimal_lambda(x, u, 20, 30, 1, 10^(-6), k);
    %
    if k > 2
        x.v_h(k+1) = min(min(x.v_h(k) + total_acceleration * dt, v_lim) , v_h_m3(x,k));
    else
        x.v_h(k+1) = min(x.v_h(k) + total_acceleration * dt, v_lim); % Speed update
    end
    x.s_h(k+1) = x.s_h(k) + x.v_h(k) * dt;              % Position update
    x.n_g(k+1) = f_update(3);                           % Gear position update
    
    px.s_p(k + 1) = px.s_p(k) + px.v_p(k)*dt;
    px.v_p(k + 1) = px.v_p(k);

    u.F_b(k + 1) = u.F_b(k);
    u.T_f(k + 1) = u.T_f(k);
    u.u_g(k + 1) = u.u_g(k);

    %%%%
end

% Plot Results
figure;
subplot(4, 1, 1);
plot(time, x.v_h, time, px.v_p,'LineWidth', 2);
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Vehicle Speed over Time');
grid on;

subplot(4, 1, 2);
plot(time, x.s_h, time, px.s_p, 'LineWidth', 2);
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

%% Offset













%% p and H Functions

% Optimal torque calculation (27)
%T_f_opt = @(k) calculate_T_f_opt(k);

% Optimal brake force calculation (28)
%F_b_opt = @(k) -p4(k)/(2*p3(k));

% Hamiltonian minimization (25)
%min_H = @(k) min(H_drive(k), H_brake(k));
% u.u_g(k) = u_g_opt(k);
% u.T_f(k) = T_f_opt(k);
% u.F_b(k) = F_b_opt(k);


% Optimal gear selection (29-30)
%u_g_opt = @(k) find_optimal_gear(k);

% Helper function for T_f optimization



function dLdv = calc_dLdv(x,k)
    % Calculate dωf/dvh at k+1
    dw_dv = 30/(pi*r_w) * I_g(x.n_g(k+1));
    dwr_dv = 2*w_r(x.v_h - v_ref);
    % Calculate fuel consumption derivative
    dmdot_dv = 0;
    for i = 0:2
        for j = 1:2  % j starts from 1 since j=0 term's derivative is 0
            dmdot_dv = dmdot_dv + ...
                kappa(i+1,j+1) * ...                    % κi,j coefficient
                u.T_f(k+1)^i * ...                      % Tf^i term
                j * ...                                 % Power rule coefficient
                (dw_dv)^j * ...                        % Chain rule term
                x.v_h(k+1)^(j-1);                      % Power rule term
        end
    end
    dLdv = dmdot_dv + dwr_dv;
end

function B = get_B(x,k)
    B = calc_dLdv(x,k)*dt;
end
function A = get_A(x,k)
    A = 1 - (rho*c_d*A_f/M)*x.v_h(k)*dt;
end
function lambda_ = get_Lambda(x,k,lambda)
    lambda_ = (lambda(k) - get_B(x,k))/get_A(x,k);
end











%% Langrange Functions
function F = calc_boundary_condition(lambda_1, N, x, u, v_f, phi, k) % #2
    % Initialize lambda array for N+1 steps
    lambda = zeros(N+1, 1);
    lambda(1) = lambda_1;
    
    % Forward simulation to get v_h(N+1)
    [x_final, lambda_final] = forward_simulation(x, u, lambda_1, N, k);
    
    % Calculate F according to equation (31)
    F = lambda_final - 2*phi*(x_final.v_h(N+1) - v_f);
end

% Function to get bounds [ΛL, ΛU] based on equations (32)-(37)
function [Lambda_L, Lambda_U] = get_lambda_bounds(x, N,v_f, phi, k) % #1
    B_min = -100;  % Initialize to conservative value
    B_max = 100;   % Initialize to conservative value
    
    % Calculate q
    q = max(abs(x.v_h(k) - v_f));
    
    % Terminal conditions for λmax and λmin
    lambda_max_N1 = 2*phi*q;
    lambda_min_N1 = -2*phi*q;
    
    % Calculate bounds using equations (36)

    lambda_max = lambda_max_N1 + N * B_max;
    lambda_min = lambda_min_N1 + N * B_min;

    % Set the upper and lower bounds for λ(1)
    Lambda_U = lambda_max;
    Lambda_L = lambda_min;
end

% Main bisection method implementation to find optimal λ(1)
function lambda_opt = find_optimal_lambda(x, u, N, v_f, phi, epsilon,k)
    % Get bounds
    [Lambda_L, Lambda_U] = get_lambda_bounds(x, N,v_f, phi,k);
    
    % Initialize bisection
    lambda_a = Lambda_L;
    lambda_b = Lambda_U;
    
    % Evaluate F at bounds
    F_a = calc_boundary_condition(lambda_a, N, x, u,v_f, phi,k);
    F_b = calc_boundary_condition(lambda_b, N, x, u,v_f, phi,k);
    
    % Check if solution exists (F(ΛL) and F(ΛU) should have opposite signs)
    if F_a * F_b >= 0
        error('No solution exists in the given interval');
    end
    
    % Bisection iteration
    while abs(lambda_b - lambda_a) > epsilon
        % Calculate midpoint
        lambda_m = (lambda_a + lambda_b) / 2;
        F_m = calc_boundary_condition(lambda_m, N, x, u, v_f, phi, k);
        
        % Update interval
        if F_m * F_a < 0
            lambda_b = lambda_m;
            F_b = F_m;
        else
            lambda_a = lambda_m;
            F_a = F_m;
        end
    end
    
    lambda_opt = (lambda_a + lambda_b) / 2;
end

% Helper function for forward simulation
function [x_final, lambda_final] = forward_simulation(x, u, lambda_1, N, k)
    % Initialize states and co-states
    lambda = zeros(N+1, 1);
    lambda(k) = lambda_1;
    x_final = x;
    
    % Forward simulation using system dynamics and co-state equations
    for n = k:(k+N)
        % Update states using optimal control (would need to implement specific control law)
        [x_final, lambda] = update_states_and_costates(x_final, u, lambda, n);
    end
    
    lambda_final = lambda;
end

% Helper function to update states and co-states
function [x, lambda] = update_states_and_costates(x, u, lambda, k)
    u.u_g(k) = u_g_opt(x, u, lambda, k);
    x.n_g(k + 1) = x.n_g(k) + u.u_g(k);
    u.T_f(k) = T_f_opt(x, u, lambda, k);
    u.F_b(k) = F_b_opt(lambda,k);
    f_update = f(x, u, k);                    
    accel = f_update(1);
    if k > 2
        x.v_h(k+1) = min(min(x.v_h(k) + accel * dt, v_lim) , v_h_m3(x,k));
    else
        x.v_h(k+1) = min(x.v_h(k) + accel * dt, v_lim); % Speed update
    end
    x.s_h(k+1) = x.s_h(k) + x.v_h(k) * dt;              % Position update
    %x.n_g(k+1) = f_update(3);                           % Gear position update
    %Need to update lead vehicle for prediction
    lambda = get_Lambda(x,k,lambda);
end



%% MAIN HELPERS
function WF = w_f(x,k)
    global I_g r_w w_f_max
    WF = min(30/(pi*r_w) * I_g(x.n_g(k)) * x.v_h(k), w_f_max);
end

function TFM = T_f_max(x,k)
    global T_peak w_f_peak
    TFM = T_peak * min(1, w_f_peak/w_f(x,k));
end

function get_accel = get_accelerations(x, k) 
    global rho A_f C_d M alpha f_rr g
    get_accel = (rho * A_f * C_d / (2 * M)) * x.v_h(k)^2 + ...   % Air resistance
                g * f_rr * cos(alpha) + ...                 % Rolling resistance
                g * sin(alpha);                             % Gravity (set to 0 for flat road)
end 

function f = vehicle_dynamics(x, u, k)
    global eta_t M r_w I_g

    % Initialize f as a column vector
    f = zeros(3, 1);
    
    % Speed update (f1)
    f(1) = (eta_t / (M * r_w)) * u.T_f(k) * I_g(x.n_g(k)) - ...
           (u.F_b(k) / M) - get_accelerations(x, k);
    
    % Position update (f2)
    f(2) = x.v_h(k);
    
    % Gear position update with bounds (f3)
    f(3) = max(1, min(x.n_g(k) + u.u_g(k), length(I_g)));
end

function val = T_w_max(x, k)
    global T_peak I_g eta_t
    val = T_peak * I_g(x.n_g(k)) * eta_t;
end
function val = v_h_m3(x, k)
    global eta_t 
    val = x.v_h(k) + eta_t * T_w_max(x,k) - get_accelerations(x, k); % (13c)
end

function val = T_max(x, k) % Need to calculate v_h_max(k + 1) from 12 - 13
    global M r_w eta_t v_h_max dt
    val = min(T_f_max(x,k), ...
    ((v_h_max(k+1) - x.v_h(k))/dt + get_accelerations(x,k)) * M*r_w/eta_t);
end


%H = @(x,u,k) p1(x,k)*u.T_f(k)^2 + p2(x,k)*u.T_f(k) + p3(x,k)*u.F_b(x,k)^2 + p4(x,k)*u.F_b(k) + p5(x,k);