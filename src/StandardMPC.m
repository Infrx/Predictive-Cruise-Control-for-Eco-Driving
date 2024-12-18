classdef StandardMPC < handle
    properties
        % System dimensions
        nx = 3;  % Number of states [v_h, s_h, n_g]
        nu = 3;  % Number of inputs [T_f, F_b, u_g]
        N = 25;  % Prediction horizon
        
        % Vehicle parameters (From the paper)
        M = 1420;          % Vehicle mass (kg) 
        rho = 1.205;       % Air density (kg/m^3)
        A_f = 1.7;         % Frontal area (m^2)
        C_d = 0.36;        % Drag coefficient
        f_rr = 0.011;      % Rolling resistance
        eta_t = 0.94;      % Drivetrain efficiency
        r_w = 0.33;        % Wheel radius (m)
        g = 9.81;          % Gravity (m/s^2)
        dt = 0.1;          % Time step (s)
        alpha = 0;         % Road slope (rad)
        
        % Controller parameters
        I_g = [17.23, 9.78, 6.42, 4.89, 4.08];  % Gear ratios
        w_r = 150;         % Reference tracking weight
        w_c = 0.4;         % Control effort weight
        v_ref = 30;        % Reference velocity (m/s)
        kappa               % Fuel consumption coefficients
        
        % Engine parameters
        T_peak = 260;      % Peak torque (Nm)
        w_f_peak = 315;    % Peak engine speed (rad/s)
        w_f_max = 733;     % Max engine speed (rad/s)
        
        % Safety parameters
        t_react = 0.5;     % Reaction time (s)
        a_h_max = 3.924;   % Max acceleration (0.4g)
        a_h_bmax = 9.81;   % Max braking decel (1.0g)
        F_b_max            % Max brake force (N)
        
        % Previous inputs for control effort calculation
        prev_T_f = 0;
        prev_gear = 1;
    end
    
    methods
        function obj = StandardMPC()
            % Initialize fuel consumption coefficients (from paper)
            obj.kappa = [0.01, 0, 0;              % κ0,j terms
                        1, 0.00005, 0.00001;      % κ1,j terms
                        0.01, 0.0006, 0];         % κ2,j terms
            
            % Calculate maximum brake force
            obj.F_b_max = obj.M * obj.a_h_bmax;
        end
        
        function [x_pred, u_opt] = solve(obj, x0, pred_vehicle)
            % Formulate and solve the optimization problem
            
            % Total number of variables: states + inputs for N steps
            nvar = obj.nx*obj.N + obj.nu*(obj.N-1);
            
            % Initial guess - all zeros
            z0 = zeros(nvar, 1);
            
            % Get constraints
            [A, b] = obj.get_inequality_constraints(x0, pred_vehicle);
            [Aeq, beq] = obj.get_equality_constraints(x0);
            [lb, ub] = obj.get_variable_bounds();
            
            % Define optimization options
            options = optimoptions('fmincon', ...
                                 'Display', 'off', ...
                                 'Algorithm', 'interior-point', ...
                                 'MaxFunctionEvaluations', 3000);
            
            % Solve optimization problem
            [z_opt, ~, exitflag] = fmincon(@(z)obj.objective_function(z, x0), ...
                                         z0, A, b, Aeq, beq, lb, ub, ...
                                         @(z)obj.nonlinear_constraints(z, x0), ...
                                         options);
            
            % Extract optimal states and inputs
            x_pred = reshape(z_opt(1:obj.nx*obj.N), [obj.nx, obj.N]);
            u_indices = obj.nx*obj.N + (1:obj.nu*(obj.N-1));
            u_opt = reshape(z_opt(u_indices), [obj.nu, obj.N-1]);
        end
        
        function [A, b] = get_inequality_constraints(obj, x0, pred_vehicle)
            % Initialize constraint matrices
            n_constraints = obj.N * 3;  % Speed, safety distance, and gear constraints
            n_variables = obj.nx*obj.N + obj.nu*(obj.N-1);
            A = zeros(n_constraints, n_variables);
            b = zeros(n_constraints, 1);
            
            row = 1;
            for k = 1:obj.N
                % Maximum velocity constraint
                A(row, obj.nx*(k-1)+1) = 1;
                b(row) = obj.v_ref;  % Use reference velocity as speed limit
                row = row + 1;
                
                % Safety distance constraint
                if k < obj.N
                    A(row, obj.nx*(k-1)+2) = 1;  % position state
                    b(row) = pred_vehicle.s_p - obj.calculate_safe_distance(x0(1), pred_vehicle.v_p);
                    row = row + 1;
                end
                
                % Gear constraint
                if k < obj.N
                    A(row, obj.nx*(k-1)+3) = 1;
                    b(row) = 5;  % Maximum gear
                    row = row + 1;
                end
            end
            
            A = A(1:row-1, :);
            b = b(1:row-1);
        end
        
        function [Aeq, beq] = get_equality_constraints(obj, x0)
            % Initial state constraints
            n_eq = obj.nx;
            n_variables = obj.nx*obj.N + obj.nu*(obj.N-1);
            Aeq = zeros(n_eq, n_variables);
            beq = zeros(n_eq, 1);
            
            % Fix initial state
            Aeq(1:obj.nx, 1:obj.nx) = eye(obj.nx);
            beq(1:obj.nx) = x0;
        end
        
        function [lb, ub] = get_variable_bounds(obj)
            % Variable bounds for states and inputs
            n_variables = obj.nx*obj.N + obj.nu*(obj.N-1);
            
            lb = -inf(n_variables, 1);
            ub = inf(n_variables, 1);
            
            % State bounds
            for k = 1:obj.N
                state_idx = (k-1)*obj.nx + (1:obj.nx);
                % Speed bounds
                lb(state_idx(1)) = 0;
                ub(state_idx(1)) = obj.v_ref;
                % Position bounds
                lb(state_idx(2)) = 0;
                % Gear bounds
                lb(state_idx(3)) = 1;
                ub(state_idx(3)) = 5;
            end
            
            % Input bounds
            input_start = obj.nx*obj.N;
            for k = 1:obj.N-1
                input_idx = input_start + (k-1)*obj.nu + (1:obj.nu);
                % Torque bounds
                lb(input_idx(1)) = 0;
                ub(input_idx(1)) = obj.T_peak;
                % Brake force bounds
                lb(input_idx(2)) = 0;
                ub(input_idx(2)) = obj.F_b_max;
                % Gear shift bounds
                lb(input_idx(3)) = -1;
                ub(input_idx(3)) = 1;
            end
        end
        
        function cost = objective_function(obj, z, x0)
            % Extract states and inputs
            states = reshape(z(1:obj.nx*obj.N), [obj.nx, obj.N]);
            inputs_idx = obj.nx*obj.N + (1:obj.nu*(obj.N-1));
            inputs = reshape(z(inputs_idx), [obj.nu, obj.N-1]);
            
            cost = 0;
            
            % Running costs
            for k = 1:obj.N-1
                % State cost (tracking)
                cost = cost + obj.w_r * (states(1,k) - obj.v_ref)^2;
                
                % Input costs
                T_f = inputs(1,k);
                F_b = inputs(2,k);
                curr_gear = max(1, min(5, round(states(3,k))));
                
                % Fuel consumption
                w_f = min(30/(pi*obj.r_w) * obj.I_g(curr_gear) * states(1,k), obj.w_f_max);
                fuel_cost = obj.calculate_fuel_rate(T_f, w_f);
                
                % Control effort
                if k > 1
                    prev_gear = max(1, min(5, round(states(3,k-1))));
                    control_cost = obj.w_c * ((T_f*obj.I_g(curr_gear) - ...
                        inputs(1,k-1)*obj.I_g(prev_gear))^2 + F_b^2);
                else
                    control_cost = obj.w_c * ((T_f*obj.I_g(curr_gear) - ...
                        obj.prev_T_f*obj.I_g(obj.prev_gear))^2 + F_b^2);
                end
                
                cost = cost + (fuel_cost + control_cost) * obj.dt;
            end
        end
        
        function [c, ceq] = nonlinear_constraints(obj, z, x0)
            % Nonlinear constraints (vehicle dynamics)
            states = reshape(z(1:obj.nx*obj.N), [obj.nx, obj.N]);
            inputs_idx = obj.nx*obj.N + (1:obj.nu*(obj.N-1));
            inputs = reshape(z(inputs_idx), [obj.nu, obj.N-1]);
            
            % Initialize
            ceq = zeros((obj.N-1)*obj.nx, 1);
            
            for k = 1:obj.N-1
                % Current state and input
                v_h = states(1,k);
                s_h = states(2,k);
                n_g = max(1, min(5, round(states(3,k))));
                T_f = inputs(1,k);
                F_b = inputs(2,k);
                
                % Next state
                v_next = states(1,k+1);
                s_next = states(2,k+1);
                
                % Acceleration components
                a_aero = (obj.rho * obj.A_f * obj.C_d / (2 * obj.M)) * v_h^2;
                a_roll = obj.g * obj.f_rr * cos(obj.alpha);
                a_grav = obj.g * sin(obj.alpha);
                
                % Dynamics constraints
                idx = (k-1)*obj.nx + (1:obj.nx);
                ceq(idx(1)) = v_next - (v_h + ((obj.eta_t/(obj.M*obj.r_w))*T_f*obj.I_g(n_g) - ...
                    F_b/obj.M - (a_aero + a_roll + a_grav))*obj.dt);
                ceq(idx(2)) = s_next - (s_h + v_h*obj.dt);
            end
            
            % No inequality constraints
            c = [];
        end
        
        function d_safe = calculate_safe_distance(obj, v_h, v_p)
            % Calculate safe following distance based on equation (10)
            s_h_br = v_h * obj.t_react + (v_h^2)/(2*obj.a_h_bmax);
            s_p_br = (v_p^2)/(2*obj.a_h_bmax);
            d_safe = max(v_h * obj.t_react, s_h_br - s_p_br);
        end
        
        function rate = calculate_fuel_rate(obj, T_f, w_f)
            % Calculate fuel consumption rate based on equation (7)
            rate = 0;
            for i = 0:2
                for j = 0:2
                    rate = rate + obj.kappa(i+1,j+1) * T_f^i * w_f^j;
                end
            end
        end
    end
end