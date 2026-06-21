clear; clc; close all;

%% Parameters
g = 9.81;

d1 = 0.0919;
a2 = 0.2385;
a3 = 0.235;

m1 = 0.20;
m2 = 0.40;
m3 = 0.30;

q_star = [0; pi/4; -pi/6];
x_star = [q_star; zeros(3,1)];

%% Robot model
L(1) = Link('revolute', 'd', d1, 'a', 0,  'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0,  'a', a3, 'alpha', 0);

L(1).m = m1;
L(1).r = [0, 0, -d1/2];
L(1).I = diag([m1*d1^2/12, m1*d1^2/12, 0]);

L(2).m = m2;
L(2).r = [-a2/2, 0, 0];
L(2).I = diag([0, m2*a2^2/12, m2*a2^2/12]);

L(3).m = m3;
L(3).r = [-a3/2, 0, 0];
L(3).I = diag([0, m3*a3^2/12, m3*a3^2/12]);

robot = SerialLink(L, ...
    'name', 'RRR-AX12A+garra', ...
    'gravity', [0; 0; -g]);

%% Equilibrium torque
Gfun = @(q) -robot.gravload(q')';
G_star = Gfun(q_star);
u_star = G_star;

%% Linearization
M_star = robot.inertia(q_star');
M_inv = inv(M_star);

dGdq = zeros(3,3);
h = 1e-6;

for i = 1:3
    qp = q_star;
    qm = q_star;

    qp(i) = qp(i) + h;
    qm(i) = qm(i) - h;

    dGdq(:,i) = (Gfun(qp) - Gfun(qm))/(2*h);
end

A = [zeros(3), eye(3);
    -M_inv*dGdq, zeros(3)];

B = [zeros(3);
     M_inv];

%% MPC model in deviation variables
C_mpc = eye(6);
D_mpc = zeros(6,3);

plant_c = ss(A, B, C_mpc, D_mpc);

Ts = 0.02;      % MPC sampling time
plant_d = c2d(plant_c, Ts);

Np = 30;        % prediction horizon
Nc = 8;         % control horizon

mpcobj = mpc(plant_d, Ts, Np, Nc);

%% Weights
% Outputs are [dq1 dq2 dq3 dqd1 dqd2 dqd3] in deviation form.
% Actually: [Δq1 Δq2 Δq3 Δdq1 Δdq2 Δdq3]
mpcobj.Weights.OutputVariables = [50 50 50 2 2 2];
mpcobj.Weights.ManipulatedVariables = [0 0 0];
mpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1];

%% Torque limits
% Example values. Adjust later based on actuator limits.
tau_min = [-1.5; -1.5; -1.5];
tau_max = [ 1.5;  1.5;  1.5];

for i = 1:3
    mpcobj.MV(i).Min = tau_min(i) - u_star(i);
    mpcobj.MV(i).Max = tau_max(i) - u_star(i);
end

%% Nominal values
mpcobj.Model.Nominal.X = zeros(6,1);
mpcobj.Model.Nominal.U = zeros(3,1);
mpcobj.Model.Nominal.Y = zeros(6,1);

%% Initial Cartesian position
p_star = [ ...
    cos(q_star(1))*(a2*cos(q_star(2)) + a3*cos(q_star(2)+q_star(3)));
    sin(q_star(1))*(a2*cos(q_star(2)) + a3*cos(q_star(2)+q_star(3)));
    d1 + a2*sin(q_star(2)) + a3*sin(q_star(2)+q_star(3)) ...
];

fprintf('q_star = \n'); disp(q_star)
fprintf('x_star = \n'); disp(x_star)
fprintf('u_star = \n'); disp(u_star)
fprintf('p_star = \n'); disp(p_star)
fprintf('MPC object created.\n');