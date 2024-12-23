%% Scenario 1 - Free Acceleration
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

%% Scenario 2 - Cruise 

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
px.s_p(2) = 20;

vh_max(1) = 30;
vh_max(2) = 30;

%% Scenario 3 - Cruise to Low Speed

x.v_h(1) = 0;
x.v_h(2) = 0;              % Initial speed (m/s)
x.s_h(1) = 0;              % Initial position (m)
x.n_g(1) = 1;              % Initial gear (starting in gear 1)
x.n_g(2) = 1;
u.T_f(1) = 10;
u.T_f(2) = 10;
u.F_b(1) = 0;
u.F_b(2) = 0;

px.v_p(2) = 5;
px.s_p(2) = 50;

vh_max(1) = 30;
vh_max(2) = 30;

%% Scenario 4 - Cruise to Full Brake

x.v_h(1) = 0;
x.v_h(2) = 0;              % Initial speed (m/s)
x.s_h(1) = 0;              % Initial position (m)
x.n_g(1) = 1;              % Initial gear (starting in gear 1)
x.n_g(2) = 1;
u.T_f(1) = 10;
u.T_f(2) = 10;
u.F_b(1) = 0;
u.F_b(2) = 0;

px.v_p(2) = 0;
px.s_p(2) = 200;

vh_max(1) = 30;
vh_max(2) = 30;