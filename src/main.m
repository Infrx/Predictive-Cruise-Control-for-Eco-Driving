% Vehicle Model Parameters
clc;
global M rho A_f C_d f_rr eta_t r_w g alpha t_react a_h_max a_h_bmax F_b_max T_peak w_f_peak w_f_max v_lim I_g v_h_max dt vh_max kappa v_ref w_r w_c  beta1 beta2 gamma1 gamma2 ;
M = 1420;                % Vehicle mass (kg)
rho = 1.205;             % Air density (kg/m^3)
A_f = 1.7;               % Frontal area (m^2)
C_d = 0.36;              % Drag coefficient (air resistance)
f_rr = 0.011;            % Rolling resistance coefficient
eta_t = 0.94;            % Drive train total efficiency
r_w = 0.33;              % Dynamic tire radius (m)
g = 9.81;                % Gravitational acceleration (m/s^2)
alpha = 0;               % Road slope (flat road)
t_react = 0.5;           % Reaction time (s)
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
Offset = 30;
N = 300 + Offset;                 % Number of time steps
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
vh_max = zeros(N, 1);
v_h_max = 30*ones(N + 1,1);
% Preceeding Vehicle States
px = struct();
px.v_p = zeros(N, 1);      % Vehicle speed (m/s)
px.s_p = zeros(N, 1);      % Vehicle position (m)
pred_px.v_p = zeros(N, 1);     
pred_px.s_p = zeros(N, 1);      
fuel_rate_ = zeros(N, 1); % Instantaneous fuel rate (kg/s)
total_fuel_ = zeros(N, 1); % Cumulative fuel consumption (kg)
beta1 = 0.5; 
beta2 = 0.5;
gamma1 = 5; 
gamma2 = 30;

% Define the control input struct u
u = struct();
u.T_f = zeros(N, 1); % Engine torque (Nm)
u.F_b = zeros(N, 1);      % Brake force (N)
u.u_g = zeros(N, 1);      % Gearshift command (shift, sustain, upshift)
% Gear ratio array (I_g) for each gear (AMT with 5 gears)
I_g = [17.23, 9.78, 6.42, 4.89, 4.08];  % Gear ratios for 5 gears

%% Scenario
x.v_h(1) = 0;
x.v_h(2) = 0;              % Initial speed (m/s)
x.s_h(1) = 0;              % Initial position (m)
x.n_g(1) = 1;              % Initial gear (starting in gear 1)
x.n_g(2) = 1;
u.T_f(1) = 10;
u.T_f(2) = 10;
u.F_b(1) = 0;
u.F_b(2) = 0;

px.v_p(2) = 10;
px.s_p(2) = 1500;

vh_max(1) = 30;
vh_max(2) = 30;
%% Fuel Rate Polynomial
kappa = [
    0.00001, 0, 0;       % κ0,j terms (constant term)
    0.00002, 0.000001, 0; % κ1,j terms (linear in T_f and w_f)
    0.000005, 0.0000001, 0 % κ2,j terms (quadratic terms)
];

TWF = @(k) [T_f(k)^0 * w_f(k)^0, T_f(k)^0 * w_f(k)^1, T_f(k)^0 * w_f(k)^2;
            T_f(k)^1 * w_f(k)^0, T_f(k)^1 * w_f(k)^1, T_f(k)^1 * w_f(k)^2;
            T_f(k)^2 * w_f(k)^0, T_f(k)^2 * w_f(k)^1, T_f(k)^2 * w_f(k)^2];
m_fuel_dot = @(k) kappa .* TWF(k); % (7)
%% Penalties 
w_r = 150;
w_c = 0.4;
v_ref = 25;
v_f = 30; % terminal speed needs to be clarified
phi = 1;
P = @(x, u, k) (u.T_f(k) * I_g(x.n_g(k)) - u.T_f(k - 1) * I_g(x.n_g(k - 1)))^2 + u.F_b(k)^2; % (6)
L = @(x, u, k) m_fuel_dot(k) + w_r*(x.v_h(k) - v_ref)^2 + w_c*P(x, u, k); % (5)
J = @(x, u, k) L(x, u, k)*dt + phi*(x.v_h(k + 1) - v_f)^2;  % (4)

%% Simulation Loop with Constraints

for k = 2:N - Offset
    % Apply physical constraints to the control inputs
    pred_px = predictionModel(x, px, k);
    vh_max(k + 1) = getVhmax(x, pred_px, k);
    % PMP STUFF
    if vh_max(k + 1) < x.v_h(k)
        u.T_f(k) = 0;
        u.F_b(k) = F_b_max/4 * (abs(vh_max(k + 1) - x.v_h(k)));
        u.u_g(k) = gearCond(x, k);
        f_upt = vehicle_dynamics(x, u, k);
        x.v_h(k + 1) = x.v_h(k) + f_upt(1)*dt;
        x.s_h(k + 1) = x.s_h(k) + x.v_h(k + 1)*dt;
        x.n_g(k + 1) = x.n_g(k) + u.u_g(k);
    else
        [L_U, L_L, F_A, F_B] = initPMP(x, u, 20, v_f, 1, k);
        L_OPT = bisection(x, u, v_f, 1, 10, L_U, L_L, F_A, F_B, k);
        lambda(k) = L_OPT;
        [u_n, x_n] = apply_control(x, u, L_OPT, k);
        x.v_h(k + 1) = x_n(1);
        x.s_h(k + 1) = x_n(2);
        x.n_g(k + 1) = x_n(3);
        u.T_f(k) = u_n(1);
        u.F_b(k) = u_n(2);
        u.u_g(k) = u_n(3);
    end
    px.s_p(k + 1) = px.s_p(k) + px.v_p(k)*dt;
    %px.v_p(k + 1) = getSpeed(px, k) % for random process
    px.v_p(k + 1) = px.v_p(k);

    [fuel_rate_(k), total_fuel_(k)] = calculate_fuel_rate(k, u, x, I_g, kappa, r_w, w_f_max, dt, total_fuel_(k-1));
    
  
end
%% Acceleration from velocity array
acceleration = diff(x.v_h)/dt;
acceleration = [acceleration(1); acceleration]; %padding
%% Plot Results
% figure;
% subplot(7, 1, 1);
% plot(time(2:(end-Offset)), x.v_h(2:(end-Offset)), time(2:(end-Offset)), px.v_p(2:(end-Offset)),'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Speed (m/s)');
% title('Vehicle Speed over Time');
% grid on;
% subplot(7, 1, 2);
% plot(time(2:(end-Offset)), acceleration(2:(end-Offset)),'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Acceleration (m/s^2)');
% title('Vehicle Acceleration over Time');
% grid on;
% subplot(7, 1, 3);
% plot(time(2:(end-Offset)), x.s_h(2:(end-Offset)), time(2:(end-Offset)), px.s_p(2:(end-Offset)), 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Position (m)');
% title('Vehicle Position over Time');
% grid on;
% 
% subplot(7, 1, 4);
% plot(time(2:(end-Offset)), x.n_g(2:(end-Offset)), 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Gear Position');
% title('Gear Position over Time');
% grid on;
% 
% subplot(7, 1, 5);
% plot(time(2:(end-Offset)), u.T_f(2:(end-Offset)), 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Engine Torque');
% title('Engine Torque over Time');
% grid on;
% 
% % Subplot 5: Fuel Rate
% subplot(7, 1, 6);
% plot(time(2:(end-Offset)), fuel_rate_(2:(end-Offset)), 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Fuel Rate (kg/s)');
% title('Fuel Rate over Time');
% grid on;
% 
% % Subplot 6: Total Fuel Consumption
% subplot(7, 1, 7);
% plot(time(2:(end-Offset)), total_fuel_(2:(end-Offset)), 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Total Fuel Consumption (kg)');
% title('Cumulative Fuel Consumption');
% grid on;
fourPlot(x, u, px, acceleration, Offset, time);
%% Functions
function value = getVhmax(x, px, k)
    global dt t_react a_h_max a_h_bmax eta_t r_w M v_lim
    c = px.s_p(k) - x.s_h(k - 1) - x.v_h(k - 1)*dt;
    vhm1 = c/t_react;
    vhm2 = a_h_max*t_react + sqrt(a_h_max^2 * t_react^2 + 2*a_h_max*c + (a_h_max/a_h_bmax)* px.v_p(k)^2);
    vhm3 = x.v_h(k - 1) + (eta_t/(M*r_w))*T_w_max(x, k) - get_accelerations(x,k);
    value = min([vhm1, vhm2, vhm3, v_lim]);
end
%% p and H Functions
function dLdv = calc_dLdv(x, u ,k)
    % Calculate dωf/dvh at k+1
    global I_g v_ref r_w w_r kappa
    dw_dv = 30/(pi*r_w) * I_g(x.n_g(k+1));
    dwr_dv = 2*w_r*(x.v_h(k + 1) - v_ref);
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

function B = get_B(x, u, k)
    global dt
    B = calc_dLdv(x, u, k)*dt;
end
function A = get_A(x, k)
    global rho C_d A_f M dt
    A = 1 - (rho*C_d*A_f/M)*x.v_h(k + 1)*dt;
end
function lambda_ = getLambda(x, u, lambda, k)
    A = get_A(x, k);
    B = get_B(x, u, k);
    lambda_ = (lambda(k) - B)/A;
end

%% Helpers
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
    global eta_t M r_w I_g dt

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
    global eta_t M r_w
    val = x.v_h(k) + eta_t/(M*r_w) * T_w_max(x,k) - get_accelerations(x, k); % (13c)
end

function val = T_max(x, k) % Need to calculate v_h_max(k + 1) from 12 - 13
    global M r_w eta_t v_h_max dt vh_max
    val = min(T_f_max(x,k), ...
    (((vh_max(k+1) - x.v_h(k))/dt + get_accelerations(x,k))* M*r_w/eta_t)) ;
end
%% P - Functions

function p = p1(x,k)
    global I_g kappa w_c
    % coefficient of T_f^2 term (κ2,j terms)

    p = kappa(3,1) + kappa(3,2)*w_f(x,k) + kappa(3,3)*w_f(x,k)^2 + w_c * I_g(x.n_g(k))^2;
end

function p = p2(x, u, lambda, k)
    % coefficient of T_f term (κ1,j terms)
    global eta_t M r_w I_g kappa w_c
    p = (kappa(2,1) + kappa(2,2)*w_f(x,k) + kappa(2,3)*w_f(x,k)^2) + ...
        lambda(k) * eta_t/(M*r_w) * I_g(x.n_g(k)) - ...
        2*w_c * I_g(x.n_g(k)) * I_g(x.n_g(k-1)) * u.T_f(k-1);
end

function p = p3()
    global w_c
    p = w_c;
end

function p = p4(lambda, k)
    global M
    p = -lambda(k)/M;
end

function p = p5(x, u, lambda, k)
    global I_g kappa v_ref w_r w_c           
    p = (kappa(1,1) + kappa(1,2)*w_f(x,k) + kappa(1,3)*w_f(x,k)^2) + ...
        w_r * (x.v_h(k) - v_ref)^2 - ...
        lambda(k) * get_accelerations(x, k) + ...
        w_c * (u.T_f(k-1) * I_g(x.n_g(k-1)))^2;
end

%% T, F, Gear and H / Functions

function T_f = T_f_opt(x, u, lambda, k)
    p1_ = p1(x, k);
    p2_ = p2(x, u, lambda, k);
    if p1_ > 0
        T_f_candidate = -p2_/(2*p1_);
        T_f = min(max(T_f_candidate, 0), T_max(x,k));
    elseif p1_ < 0
        if -p2_/(2*p1_) <= T_max(x,k)
            T_f = T_max(x,k);
        else
            T_f = 0;
        end
    elseif p1_ == 0
        if p2_ >= 0
            T_f = 0;
        else
            T_f = T_max(x,k);
        end
    end
end
function F_b = F_b_opt(lambda,k)
    global F_b_max
    p3_ = p3();
    p4_ = p4(lambda, k);
    if p3_ > 0
        F_b_candidate = -p4_/(2*p3_);
        F_b = min(max(F_b_candidate, 0), F_b_max);
    end
end
% Helper function for gear optimization


function [val, idx] = H(x, u, lambda, T_f, F_b, k)
    %  Hdrive and H brake sepeartly and take min
    % optimal control must be Tf 0 or 0 Fb
    H_drive = p1(x, k)*T_f^2 + p2(x, u, lambda, k)*T_f + p5(x, u, lambda, k);
    H_brake = p3()*F_b^2 + p4(lambda, k)*F_b + p5(x, u, lambda, k);
    [val, idx] = min([H_drive, H_brake]);
end

%% Langrange Functions
% Function to get bounds [ΛL, ΛU] based on equations (32)-(37)

function [Lambda_U, Lambda_L, F_A, F_B] = initPMP(x, u, N,v_f, phi, k)
    B_min = -100000;  % Initialize to conservative value
    B_max =  100000;  

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
    [F_A, ~] = forward_simulation(x, u, v_f, phi, Lambda_L, N, k);
    [F_B, ~] = forward_simulation(x, u, v_f, phi, Lambda_U, N, k);

    if F_A*F_B > 0
        fprintf("Sign error F_A vs F_B");
    end
end

function [F, lambda_final] = forward_simulation(x, u, v_f, phi, lambda_1, N, k)
    % Initialize costates for prediction horizon
    lambda = zeros(k + N + 1, 1);
    lambda(k) = lambda_1;
    x_sim = x;
    % Forward simulation from current time k to k+N
    for n = k:(k+N)
        % Get optimal control using PMP
        u_opt = calculate_optimal_control(x_sim, u, lambda, n);
        % Update state
        [u, x_sim] = update_states(x_sim, u, u_opt, n);
        
        % Update costate

        lambda(n + 1) = getLambda(x_sim, u, lambda, n);
    end
    
    % Calculate F using terminal values
    F = lambda(k + N + 1) - 2*phi*(x_sim.v_h(k + N + 1) - v_f);
    lambda_final = lambda(k + N + 1);
end

function u_opt = calculate_optimal_control(x_sim, u, lambda, k)
    %given current states, and lambda find optimal control input u_opt
    %current gear, current gear + 1, current gear - 1
    %find T and F optimal for each, 
    %then find the optimal  (T,F) combination w.r.t gear position
    %return u_opt = [T_opt, F_opt, u_g_opt]
    current_gear = x_sim.n_g(k);
    u_g = gearCond(x_sim, k);
    [~, idx] = H(x_sim, u, lambda, T_f_opt(x_sim, u, lambda,k), F_b_opt(lambda,k), k);
    if idx == 1
        u_opt = [T_f_opt(x_sim, u, lambda,k) 0 u_g];
    elseif idx == 2
        u_opt = [0 F_b_opt(lambda,k) u_g];
    else
        fprintf("Unknown index at opt cont\n");
    end   



end

function [u, x_sim] = update_states(x_sim, u, u_opt, n) % need T_max vhmax dont forget
    global eta_t M r_w I_g dt
    % Speed update 
    x_sim.v_h(n + 1) = x_sim.v_h(n) + ((eta_t / (M * r_w)) * u_opt(1) * I_g(x_sim.n_g(n)) - ...
           (u_opt(2) / M) - get_accelerations(x_sim, n))*dt;
    
    % Position update 
    x_sim.s_h(n + 1) = x_sim.s_h(n) + x_sim.v_h(n)*dt;
    
    % Gear position update with bounds 
    x_sim.n_g(n + 1) = max(1, min(x_sim.n_g(n) + u_opt(3), length(I_g)));

    u.T_f(n) = u_opt(1);
    u.F_b(n) = u_opt(2);
    u.u_g(n) = u_opt(3);
end
function lambda_optimal = bisection(x, u, v_f, phi, N, L_U, L_L, F_A, F_B, k)
    r = 1;
    it = 20;
    lambda = (L_U + L_L)/2;
    [F, ~]= forward_simulation(x, u, v_f, phi, lambda, N, k);
    while true
        if abs(F) <= 10^(-6)
            break
        end
        
        if F*F_A < 0
            L_U = lambda;
        else
            L_L = lambda;
        end
        r = r + 1;
        lambda = (L_U + L_L)/2;
        
        [F, ~] = forward_simulation(x, u, v_f, phi, lambda, N, k);
        [F_A, ~]= forward_simulation(x, u, v_f, phi, L_L, N, k);
    end

    lambda_optimal = lambda;
end

function [u_curr, x_next] = apply_control(x, u, lambda_opt, k)
    lambda = lambda_opt*ones(k,1);
    x_sim = x;
    u_opt = calculate_optimal_control(x, u, lambda, k);
    [~, x_sim] = update_states(x_sim, u, u_opt, k);
    u_curr = [u_opt(1) u_opt(2) u_opt(3)];
    x_next = [x_sim.v_h(k + 1) x_sim.s_h(k + 1) x_sim.n_g(k + 1)];
end

%% old optimal control
% function u_opt = calculate_optimal_control(x_sim, u, lambda, k)
%     %given current states, and lambda find optimal control input u_opt
%     %current gear, current gear + 1, current gear - 1
%     %find T and F optimal for each, 
%     %then find the optimal  (T,F) combination w.r.t gear position
%     %return u_opt = [T_opt, F_opt, u_g_opt]
%     current_gear = x_sim.n_g(k);
%     idx_up = 0;
%     idx_sus = 0;
%     idx_down = 0;
%     if current_gear == 1
%         [H_sus, idx_sus] = H(x_sim, u, lambda, T_f_opt(x_sim, u, lambda,k), F_b_opt(lambda,k), k);
%         x_sim.n_g(k) = x_sim.n_g(k) + 1;
%         [H_up, idx_up] = H(x_sim, u, lambda, T_f_opt(x_sim, u, lambda,k), F_b_opt(lambda,k), k);
%         [~, idx] = min([H_sus, H_up]);
%         u_g = idx - 1;
%         x_sim.n_g(k) = current_gear;
%         % optimal control must be [Tf 0] or [0 F_b]
%         if idx == 1
%             if idx_sus == 1
%                 u_opt = [T_f_opt(x_sim, u, lambda,k) 0 u_g];
%             elseif idx_sus == 2
%                 u_opt = [0 F_b_opt(lambda,k) u_g];
%             else
%                 fprintf("Unknown index at opt cont\n");
%             end
%         elseif idx == 2
%             if idx_up == 1
%                 u_opt = [T_f_opt(x_sim, u, lambda,k) 0 u_g];
%             elseif idx_up == 2
%                 u_opt = [0 F_b_opt(lambda,k) u_g];
%             else
%                 fprintf("Unknown index at opt cont\n");
%             end
%         end
%         %u_opt = [T_f_opt(x_sim, u, lambda,k) F_b_opt(lambda,k) u_g];
%         return
%     end
%     if current_gear == 5
%         x_sim.n_g(k) = x_sim.n_g(k) - 1;
%         [H_down, idx_down] = H(x_sim, u, lambda, T_f_opt(x_sim, u, lambda, k), F_b_opt(lambda, k), k);
%         x_sim.n_g(k) = current_gear;
%         [H_sus, idx_sus] = H(x_sim, u, lambda, T_f_opt(x_sim, u, lambda, k), F_b_opt(lambda ,k), k);
%         [~, idx] = min([H_down, H_sus]);
%         u_g = idx - 2; % Maps [1,2] to [-1,0]
%         x_sim.n_g(k) = current_gear;
%         if idx == 1
%             if idx_down == 1
%                 u_opt = [T_f_opt(x_sim, u, lambda,k) 0 u_g];
%             elseif idx_down == 2
%                 u_opt = [0 F_b_opt(lambda,k) u_g];
%             else
%                 fprintf("Unknown index at opt cont\n");
%             end
%         elseif idx == 2
%             if idx_sus == 1
%                 u_opt = [T_f_opt(x_sim, u, lambda,k) 0 u_g];
%             elseif idx_sus == 2
%                 u_opt = [0 F_b_opt(lambda,k) u_g];
%             else
%                 fprintf("Unknown index at opt cont\n");
%             end
%         end
%         %u_opt = [T_f_opt(x_sim, u, lambda, k) F_b_opt(lambda, k) u_g];
%         return
%     end
%     if (current_gear > 1 && current_gear < 5)
%         x_sim.n_g(k) = x_sim.n_g(k) - 1;
%         [H_down, idx_down] = H(x_sim, u, lambda, T_f_opt(x_sim, u, lambda,k), F_b_opt(lambda,k), k);
%         x_sim.n_g(k) = current_gear;
%         [H_sus, idx_sus] = H(x_sim, u, lambda, T_f_opt(x_sim, u, lambda,k), F_b_opt(lambda,k), k);
%         x_sim.n_g(k) = x_sim.n_g(k) + 1;
%         [H_up, idx_up] = H(x_sim, u, lambda, T_f_opt(x_sim, u, lambda,k), F_b_opt(lambda,k), k);
%         [~, idx] = min([H_down, H_sus, H_up]);
%         u_g = idx - 2; % Maps [1,2,3] to [-1,0,1]
%         x_sim.n_g(k) = current_gear;
%         if idx == 1
%             if idx_down == 1
%                 u_opt = [T_f_opt(x_sim, u, lambda,k) 0 u_g];
%             elseif idx_down == 2
%                 u_opt = [0 F_b_opt(lambda,k) u_g];
%             else
%                 fprintf("Unknown index at opt cont\n");
%             end
%         elseif idx == 2
%             if idx_sus == 1
%                 u_opt = [T_f_opt(x_sim, u, lambda,k) 0 u_g];
%             elseif idx_sus == 2
%                 u_opt = [0 F_b_opt(lambda,k) u_g];
%             else
%                 fprintf("Unknown index at opt cont\n");
%             end
%         elseif idx == 3
%             if idx_up == 1
%                 u_opt = [T_f_opt(x_sim, u, lambda,k) 0 u_g];
%             elseif idx_up == 2
%                 u_opt = [0 F_b_opt(lambda,k) u_g];
%             else
%                 fprintf("Unknown index at opt cont\n");
%             end
%         end
%         %u_opt = [T_f_opt(x_sim, u, lambda,k) F_b_opt(lambda,k) u_g];
%         return
%     end
% end
% 
% kappa = [0,0, 0;    
%         0.001, 0.001, 0; 
%         0.1, 0, 0];

function val = gearCond(x, k)
    current_gear = x.n_g(k);
    speed = x.v_h(k);  % Current speed in m/s
    
    % Speed thresholds in m/s (converted from km/h)
    upshift_speeds = [20/3.6, 40/3.6, 60/3.6, 80/3.6, inf];  % When to shift up
    downshift_speeds = [15/3.6, 35/3.6, 55/3.6, 75/3.6, inf]; % When to shift down
    
    if current_gear == 1
        if speed >= upshift_speeds(1)
            val = +1;  % Upshift
        else
            val = 0;  % Stay in current gear
        end
        
    elseif current_gear == 2
        if speed >= upshift_speeds(2)
            val = +1;  % Upshift
        elseif speed < downshift_speeds(1)
            val = -1;  % Downshift
        else
            val = 0;  % Stay in current gear
        end
        
    elseif current_gear == 3
        if speed >= upshift_speeds(3)
            val = +1;  % Upshift
        elseif speed < downshift_speeds(2)
            val = -1;  % Downshift
        else
            val = 0;  % Stay in current gear
        end
        
    elseif current_gear == 4
        if speed >= upshift_speeds(4)
            val = +1;  % Upshift
        elseif speed < downshift_speeds(3)
            val = -1;  % Downshift
        else
            val = 0;  % Stay in current gear
        end
        
    elseif current_gear == 5
        if speed < downshift_speeds(4)
            val = -1;  % Downshift
        else
            val = 0;  % Stay in current gear
        end
    end
end
function [fuel_rate, total_fuel] = calculate_fuel_rate(k, u, x, I_g, kappa, r_w, w_f_max, dt, total_fuel_prev)
  
    % Compute fuel rate using the polynomial form
    fuel_rate = 0;
    for i = 0:2
        for j = 0:2
            fuel_rate = fuel_rate + kappa(i+1, j+1) * (u.T_f(k)^i) * (w_f(x,k)^j);
        end
    end
    
    % Accumulate total fuel consumption
    total_fuel = total_fuel_prev + fuel_rate * dt/1000;
end
function px = predictionModel(x, px, k)
    
   global dt beta1 beta2 gamma1 gamma2
   if k == 2
   a=1;
   else
   a=0;
   end
   
   if ((px.v_p(k-1)-px.v_p(k+a-2))/dt) <= 0
   zeta = 1 / (1 + exp(-beta1 * (px.v_p(k-1) - gamma1))); 
   else 
   zeta = 1 / (1 + exp(beta2 * (px.v_p(k-1) - gamma2)));
   end
    
   px.v_p(k) = px.v_p(k-1) + (px.v_p(k-1)-px.v_p(k+a-2)) * zeta * dt;
    
   px.s_p(k)= px.s_p(k-1) + px.v_p(k-1) * dt + 0.5 * (px.v_p(k-1)-px.v_p(k+a-2)) * zeta * dt^2;
    
end
function val = getSpeed(px, k)
    current_speed = px.v_p(k);
    val = current_speed + current_speed*sin(k/10)/100 + 0.1*randn();
    val = max(0, min(val, current_speed*1.2));
end