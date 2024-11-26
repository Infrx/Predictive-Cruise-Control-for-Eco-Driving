%% Langrange Functions
% Function to get bounds [ΛL, ΛU] based on equations (32)-(37)
PCC = 5;
function [Lambda_U, Lambda_L, F_A, F_B] = initPMP(x, N,v_f, phi, k)
    B_min = -100;  % Initialize to conservative value
    B_max = 100;   % Initialize to conservative value
    
    % Calculate q
    q = 15; %max(abs(x.v_h(k) - v_f)); % Let it be 15 for test
    
    % Terminal conditions for λmax and λmin
    lambda_max_N1 = 2*phi*q;
    lambda_min_N1 = -2*phi*q;
    
    % Calculate bounds using equations (36)

    lambda_max = lambda_max_N1 + N * B_max;
    lambda_min = lambda_min_N1 + N * B_min;

    % Set the upper and lower bounds for λ(1)
    Lambda_U = lambda_max;
    Lambda_L = lambda_min;

    F_A = forward_simulation(x, v_f, phi, Lambda_U, N, k);
    F_B = forward_simulation(x, v_f, phi, Lambda_L, N, k);
end

function [F, lambda_final] = forward_simulation(x, v_f, phi, lambda_1, N, k)
    % Initialize costates for prediction horizon
    lambda = zeros(k + N + 1, 1);
    lambda(k) = lambda_1;
    x_sim = x;
    
    % Forward simulation from current time k to k+N
    for n = k:(k+N)
        % Get optimal control using PMP
        u_opt = calculate_optimal_control(x_sim, lambda, n);
        
        % Update state
        x_sim = update_states(x_sim, u_opt, n);
        
        % Update costate
        lambda(n + 1) = getLambda(x_sim, lambda, n);
    end
    
    % Calculate F using terminal values
    F = lambda(k + N + 1) - 2*phi*(x_sim.v_h(k + N + 1) - v_f);
    lambda_final = lambda(k + N + 1);
end

function u_opt = calculate_optimal_control(x_sim, lambda, k)
    %given current states, and lambda find optimal control input u_opt
    %current gear, current gear + 1, current gear - 1
    %find T and F optimal for each, 
    %then find the optimal  (T,F) combination w.r.t gear position
    %return u_opt = [T_opt, F_opt, u_g_opt]
    current_gear = x_sim.n_g(k);
    if current_gear == 1
    H_sus = H(x, lambda, T_f_opt(x_sim,lambda,k), F_b_opt(lambda,k), k);
    x_sim.n_g(k) = x_sim.n_g(k) + 1;
    H_up = H(x, lambda, T_f_opt(x_sim,lambda,k), F_b_opt(lambda,k), k);
    [~, idx] = min([H_sus, H_up]);
    u_g = idx - 1;
    x_sim.n_g(k) = current_gear + u_g;
    u_opt = [T_f_opt(x_sim,lambda,k) F_b_opt(lambda,k) u_g];
    return
    end
    if current_gear == 5
    x_sim.n_g(k) = x_sim.n_g(k) - 1;
    H_down = H(x_sim, lambda, T_f_opt(x_sim,lambda,k), F_b_opt(lambda,k), k);
    x_sim.n_g(k) = current_gear;
    H_sus = H(x_sim, lambda, T_f_opt(x_sim,lambda,k), F_b_opt(lambda,k), k);
    [~, idx] = min([H_down, H_sus]);
    u_g = idx - 2; % Maps [1,2] to [-1,0]
    x_sim.n_g(k) = current_gear + u_g;
    u_opt = [T_f_opt(x_sim,lambda,k) F_b_opt(lambda,k) u_g];
    return
    end
    if current_gear > 1 && current_gear < 5
    x_sim.n_g(k) = x_sim.n_g(k) - 1;
    H_down = H(x_sim, lambda, T_f_opt(x_sim,lambda,k), F_b_opt(lambda,k), k);
    x_sim.n_g(k) = current_gear;
    H_sus = H(x_sim, lambda, T_f_opt(x_sim,lambda,k), F_b_opt(lambda,k), k);
    x_sim.n_g(k) = x_sim.n_g(k) + 1;
    H_up = H(x_sim, lambda, T_f_opt(x_sim,lambda,k), F_b_opt(lambda,k), k);
    [~, idx] = min([H_down, H_sus, H_up]);
    u_g = idx - 2; % Maps [1,2,3] to [-1,0,1]
    x_sim.n_g(k) = current_gear + u_g;
    u_opt = [T_f_opt(x_sim,lambda,k) F_b_opt(lambda,k) u_g];
    return
    end
end

function x_sim = update_states(x_sim, u_opt, n) % need T_max vhmax dont forget
    global eta_t M r_w I_g dt
    % Speed update 
    x_sim.v_h(n + 1) = (eta_t / (M * r_w)) * u_opt(1) * I_g(x.n_g(n)) - ...
           (u_opt(2) / M) - get_accelerations(x, n);
    
    % Position update 
    x_sim.s_h(n + 1) = x_sim.s_h(n) + x_sim.v_h(n)*dt;
    
    % Gear position update with bounds 
    x_sim.n_g(n + 1) = max(1, min(x.n_g(k) + u_opt(3), length(I_g)));
end

%% P - FUNCTIONS

function p = p1(x,k)
    global I_g 
    % coefficient of T_f^2 term (κ2,j terms)
    w_c = 0.5;  % Make sure these global variables are accessible
    kappa = [0.1, 0.01, 0.001;    
             0.05, 0.005, 0.0005; 
             0.01, 0.001, 0.0001];
    
    p = kappa(3,1) + kappa(3,2)*w_f(x,k) + kappa(3,3)*w_f(x,k)^2 + w_c * I_g(x.n_g(k))^2;
end

function p = p2(x, lambda, k)
    % coefficient of T_f term (κ1,j terms)
    global eta_t M r_w I_g
    w_c = 0.5;
    kappa = [0.1, 0.01, 0.001;    
             0.05, 0.005, 0.0005; 
             0.01, 0.001, 0.0001];
    p = (kappa(2,1) + kappa(2,2)*w_f(x,k) + kappa(2,3)*w_f(x,k)^2) + ...
        lambda(k) * eta_t/(M*r_w) * I_g(x.n_g(k)) - ...
        2*w_c * I_g(x.n_g(k)) * I_g(x.n_g(k-1)) * u.T_f(k-1);
end

function p = p3()
    w_c = 0.5;
    p = w_c;
end

function p = p4(lambda, k)
    global M
    p = -lambda(k)/M;
end

function p = p5(x, lambda, k)
    w_r = 1;
    v_ref = 30;
    w_c = 0.5;
    kappa = [0.1, 0.01, 0.001;    
             0.05, 0.005, 0.0005; 
             0.01, 0.001, 0.0001];
             
    p = (kappa(1,1) + kappa(1,2)*w_f(x,k) + kappa(1,3)*w_f(x,k)^2) + ...
        w_r * (x.v_h(k) - v_ref)^2 - ...
        lambda(k) * get_accelerations(x, k) + ...
        w_c * (u.T_f(k-1) * I_g(x.n_g(k-1)))^2;
end

%% T, F, Gear and H / Functions

function T_f = T_f_opt(x, lambda, k)
    p1_ = p1(x,k);
    p2_ = p2(x,lambda,k);
    if p1_ > 0
        T_f_candidate = -p2_/(2*p1_);
        T_f = min(max(T_f_candidate, 0), T_max(x,k));
    end
    if p1_ < 0
        if -p2_/(2*p1_) <= T_max(x,k)
            T_f = T_max(x,k);
        else
            T_f = 0;
        end
    end
    if p1_ == 0
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
    if p3() > 0
        F_b_candidate = -p4_/(2*p3_);
        F_b = min(max(F_b_candidate, 0), F_b_max);
    end
end
% Helper function for gear optimization
function u_g = u_g_opt(x, u, lambda, k)
    H_up = Inf;
    H_sus = Inf;
    H_down = Inf;
    gear_temp = x.n_g(k);
    if gear_temp == 1
       H_down = Inf;
    else
       x.n_g(k) = x.n_g(k) - 1;
       u.T_f(k) = T_f_opt(x, u, lambda, k);
       u.F_b(k) = F_b_opt(lambda,k);
       H_down = H(x, k, lambda, u);
    end
    x.n_g(k) = gear_temp;
    if gear_temp == 5
       H_up = Inf;
    else
       x.n_g(k) = x.n_g(k - 1) + 1;
       u.T_f(k) = T_f_opt(x, u, lambda, k);
       u.F_b(k) = F_b_opt(lambda, k);
       H_up = H(x, k, lambda, u);
    end
    x.n_g(k) = gear_temp;
    u.T_f(k) = T_f_opt(x, u, lambda, k);
    u.F_b(k) = F_b_opt(lambda, k);
    H_sus = H(x, k, lambda, u);

    [~, idx] = min([H_down, H_sus, H_up]);
    u_g = idx - 2;
end

function val = H(x, lambda, T_f, F_b, k)
    val = p1(x, k)*T_f^2 + p2(x, lambda, k)*T_f(k) + p3()*F_b^2 + p4(lambda, k)*F_b + p5(x, lambda, k);
end