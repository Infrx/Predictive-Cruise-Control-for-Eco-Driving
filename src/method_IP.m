% Simulation using StandardMPC
clc; close all;

% Simulation parameters

Offset = 30;
sim_steps = 300+Offset;
dt = 0.1;

% Initialize MPC controller
mpc = StandardMPC_IP();

% Initialize state histories
x.v_h = zeros(sim_steps, 1);
x.s_h = zeros(sim_steps, 1);
x.n_g = ones(sim_steps, 1);
u.T_f = zeros(sim_steps, 1);
u.F_b = zeros(sim_steps, 1);
u.u_g = zeros(sim_steps, 1);
vh_max = zeros(sim_steps, 1);

% Initialize preceding vehicle
px.s_p = zeros(sim_steps, 1);
px.v_p = zeros(sim_steps, 1);
%N = 25;
% Initial conditions

pred_px.v_p = zeros(sim_steps-1-30, 1);     
pred_px.s_p = zeros(sim_steps-1-30, 1); 
fuel_rate = zeros(sim_steps, 1); % Instantaneous fuel rate (kg/s)
total_fuel = zeros(sim_steps, 1); % Cumulative fuel consumption (kg)
beta1 = 0.5; 
beta2 = 0.5;
gamma1 = 5; 
gamma2 = 30;

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
% Main simulation loop

for k = 2:sim_steps-Offset
    % Current state for MPC
    x0 = [x.v_h(k); x.s_h(k); x.n_g(k)];
    
    % Current preceding vehicle state
    pred_px = predictionModel(x, px, k);
    pred_vehicle.s_p = pred_px.s_p(k);
    pred_vehicle.v_p = pred_px.v_p(k);
    
    % Get optimal control input from MPC
    
    % PMP STUFF
        [x_pred, u_opt, fuel_rate(k)] = mpc.solve(x0, pred_vehicle);
        % Apply first control input
        total_fuel(k) = total_fuel(k - 1) + fuel_rate(k)*dt/1000;
        u.T_f(k) = u_opt(1,1);
        u.F_b(k) = u_opt(2,1);
        u.u_g(k) = gearCond(x, k);  % Round gear shift command
        
        % Update states using vehicle dynamics
        % Speed update
        accel = (mpc.eta_t/(mpc.M*mpc.r_w)) * u.T_f(k) * mpc.I_g(x.n_g(k)) - ...
                (u.F_b(k)/mpc.M) - ...
                ((mpc.rho * mpc.A_f * mpc.C_d)/(2 * mpc.M)) * x.v_h(k)^2 - ...
                mpc.g * mpc.f_rr;
        
        x.v_h(k+1) = x.v_h(k) + accel * dt;
        x.s_h(k+1) = x.s_h(k) + x.v_h(k) * dt;
        x.n_g(k+1) = max(1, min(5, x.n_g(k) + u.u_g(k)));
    % Update preceding vehicle
    px.s_p(k+1) = px.s_p(k) + px.v_p(k) * dt;
    px.v_p(k+1) = px.v_p(k);  % constant velocity
end


u_IP = u;
x_IP = x;
TF_IP = total_fuel;






% Plot results
t = 0:dt:(sim_steps-1)*dt;
% 
% figure('Position', [100 100 800 600]);
% 
% % Plot velocity
% subplot(4,1,1);
% plot(t, x.v_h, 'b-', 'LineWidth', 2);
% hold on;
% yline(mpc.v_ref, 'r--');
% ylabel('Velocity (m/s)');
% title('Vehicle Velocity Profile');
% legend('Actual', 'Reference');
% grid on;
% 
% % Plot position
% subplot(4,1,2);
% plot(t, x.s_h, 'b-', 'LineWidth', 2);
% hold on;
% plot(t, px.s_p, 'r--');
% ylabel('Position (m)');
% title('Vehicle Position Profile');
% legend('Host Vehicle', 'Preceding Vehicle');
% grid on;
% 
% % Plot gear
% subplot(4,1,3);
% plot(t, x.n_g, 'b-', 'LineWidth', 2);
% ylabel('Gear Position');
% title('Gear Profile');
% ylim([0.5 5.5]);
% yticks(1:5);
% grid on;
% 
% % Plot control inputs
% subplot(4,1,4);
% plot(t, u.T_f, 'b-', 'LineWidth', 2);
% hold on;
% plot(t, u.F_b, 'r-', 'LineWidth', 2);
% ylabel('Control Inputs');
% xlabel('Time (s)');
% legend('Engine Torque (Nm)', 'Brake Force (N)');
% title('Control Inputs');
% grid on;


%% functions
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

