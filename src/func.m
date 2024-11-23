%% Corrected p coefficient 
% p1: coefficient of T_f^2 term (κ2,j terms)
p1 = @(k) kappa(3,1) + kappa(3,2)*w_f(k) + kappa(3,3)*w_f(k)^2 + w_c * I_g(x.n_g(k))^2;

% p2: coefficient of T_f term (κ1,j terms)
p2 = @(k) (kappa(2,1) + kappa(2,2)*w_f(k) + kappa(2,3)*w_f(k)^2) + ...
          lambda(k) * eta_t/(M*r_w) * I_g(x.n_g(k)) - ...
          2*w_c * I_g(x.n_g(k)) * I_g(x.n_g(k-1)) * u.T_f(k-1);

% p3 and p4 remain the same
p3 = @(k) w_c;
p4 = @(k) -lambda(k)/M;

% p5: constant term (κ0,j terms)
p5 = @(k) (kappa(1,1) + kappa(1,2)*w_f(k) + kappa(1,3)*w_f(k)^2) + ...
          w_r * (x.v_h(k) - v_ref)^2 - ...
          lambda(k) * a_k(k) + ...
          w_c * (u.T_f(k-1) * I_g(x.n_g(k-1)))^2;
%%
% Equations 23-30 as anonymous functions

% Basic Hamiltonian (23)
H = @(k) p1(k)*u.T_f(k)^2 + p2(k)*u.T_f(k) + p3(k)*u.F_b(k)^2 + p4(k)*u.F_b(k) + p5(k);

% Drive and brake mode Hamiltonians (24)
H_drive = @(k) p1(k)*u.T_f(k)^2 + p2(k)*u.T_f(k) + p5(k);
H_brake = @(k) p3(k)*u.F_b(k)^2 + p4(k)*u.F_b(k) + p5(k);

% Maximum torque constraint (26)
T_max = @(k) min(T_f_max(k), ...
    ((v_h_max(k+1) - x.v_h(k))/dt + get_accelerations(x,k)) * M*r_w/eta_t);

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

function T_f = T_f_opt(k)
    if p1(k) > 0
        T_f_candidate = -p2(k)/(2*p1(k));
        T_f = min(max(T_f_candidate, 0), T_max(k));
    end
    if p1(k) < 0
        if -p2(k)/(2*p1(k)) <= T_max(k)
            T_f = T_max(k);
        else
            T_f = 0;
        end
    end
    if p1(k) == 0
        if p2(k) >= 0
            T_f = 0;
        else
            T_f = T_max(k);
        end
    end
end
function F_b = F_b_opt(k)
    if p3(k) > 0
        F_b_candidate = -p4(k)/(2*p3(k));
        F_b = min(max(F_b_candidate, 0), F_b_max);
    end
end
% Helper function for gear optimization
function u_g = u_g_opt(k)
    H_up = Inf;
    H_sus = Inf;
    H_down = Inf;
    gear_temp = x.n_g;
    if gear_temp == 1
       H_down = Inf;
    else
       x.n_g = x.n_g - 1;
       H_down = H(k);
    end
    x.n_g = gear_temp;
    if gear_temp == 5
       H_up = Inf;
    else
       x.n_g = x.n_g + 1;
       H_up = H(k);
    end
    x.n_g = gear_temp;
    H_sus = H(k);

    [~, idx] = min([H_down, H_sus, H_up]);
    u_g = idx - 2;
end

function dLdv = calc_dLdv(k)

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

function B = get_B(k)
    B = calc_dLdv(k)*dt;
end
function A = get_A(k)
    A = 1 - (rho*c_d*A_f/M)*x.v_h(k)*dt;
end
function lambda_ = get_Lambda(k)
    lambda_ = (lambda(k) - get_B(k))/get_A(k);
end