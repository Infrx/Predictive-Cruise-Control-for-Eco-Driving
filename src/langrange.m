% Function to calculate boundary condition F(λ(1)) from equation (31)
PCC = 1;
function F = calc_boundary_condition(lambda_1, N, x, v_f, phi) % #2
    % Initialize lambda array for N+1 steps
    lambda = zeros(N+1, 1);
    lambda(1) = lambda_1;
    
    % Forward simulation to get v_h(N+1)
    [x_final, lambda_final] = forward_simulation(x, lambda_1, N);
    
    % Calculate F according to equation (31)
    F = lambda_final - 2*phi*(x_final.v_h(N+1) - v_f);
end

% Function to get bounds [ΛL, ΛU] based on equations (32)-(37)
function [Lambda_L, Lambda_U] = get_lambda_bounds(x, N, phi, k) % #1
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
function lambda_opt = find_optimal_lambda(x, N, v_f, phi, epsilon)
    % Get bounds
    [Lambda_L, Lambda_U] = get_lambda_bounds(x, N, phi);
    
    % Initialize bisection
    lambda_a = Lambda_L;
    lambda_b = Lambda_U;
    
    % Evaluate F at bounds
    F_a = calc_boundary_condition(lambda_a, N, x, v_f, phi);
    F_b = calc_boundary_condition(lambda_b, N, x, v_f, phi);
    
    % Check if solution exists (F(ΛL) and F(ΛU) should have opposite signs)
    if F_a * F_b >= 0
        error('No solution exists in the given interval');
    end
    
    % Bisection iteration
    while abs(lambda_b - lambda_a) > epsilon
        % Calculate midpoint
        lambda_m = (lambda_a + lambda_b) / 2;
        F_m = calc_boundary_condition(lambda_m, N, x, v_f, phi);
        
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
function [x_final, lambda_final] = forward_simulation(x, lambda_1, N)
    % Initialize states and co-states
    lambda = zeros(N+1, 1);
    lambda(1) = lambda_1;
    x_final = x;
    
    % Forward simulation using system dynamics and co-state equations
    for k = 1:N
        % Update states using optimal control (would need to implement specific control law)
        [x_final, lambda] = update_states_and_costates(x_final, lambda, k);
    end
    
    lambda_final = lambda(N+1);
end

% Helper function to update states and co-states
function [x, lambda] = update_states_and_costates(x, lambda, k)
    % This function would implement equations (21a) and (21b)
    % and the state updates using the optimal control law
    % Specific implementation depends on your vehicle model and control law
    
    % Would need to implement:
    % 1. Calculate optimal control using current lambda
    % 2. Update states using optimal control
    % 3. Update co-states using equations (21a) and (21b)
end