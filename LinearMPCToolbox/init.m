%% ========================================================================
%  SCRIPT DE INICIALIZACAO - MPC LINEAR (RRR + AX-12A)
%  Baseado em setup_RRR_MPC.m - Rodar ANTES de abrir o Simulink
% ========================================================================
clear; clc; close all;

%% --- 1. Parametros fisicos ---
g  = 9.81;
d1 = 0.0919;
a2 = 0.2385;
a3 = 0.235;
m1 = 0.20; m2 = 0.40; m3 = 0.30;

q_star = [0; pi/4; -pi/6];
x_star = [q_star; zeros(3,1)];

%% --- 2. Robot (RTB) ---
L(1) = Link('revolute', 'd', d1, 'a', 0,  'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0,  'a', a3, 'alpha', 0);
L(1).m = m1; L(1).r = [0, 0, -d1/2];   L(1).I = diag([m1*d1^2/12, m1*d1^2/12, 0]);
L(2).m = m2; L(2).r = [-a2/2, 0, 0];   L(2).I = diag([0, m2*a2^2/12, m2*a2^2/12]);
L(3).m = m3; L(3).r = [-a3/2, 0, 0];   L(3).I = diag([0, m3*a3^2/12, m3*a3^2/12]);

robot = SerialLink(L, 'name', 'RRR-AX12A+garra', 'gravity', [0; 0; -g]);
assignin('base','robot',robot);

%% --- 3. Torque de equilibrio ---
Gfun = @(q) -robot.gravload(q')';
u_star = Gfun(q_star);

assignin('base','q_star',q_star);
assignin('base','x_star',x_star);
assignin('base','u_star',u_star);

%% --- 4. Linearizacao (Jacobiano numerico de G(q)) ---
M_star = robot.inertia(q_star');
M_inv  = inv(M_star);

dGdq = zeros(3,3);
h = 1e-6;
for i = 1:3
    qp = q_star; qm = q_star;
    qp(i) = qp(i) + h;
    qm(i) = qm(i) - h;
    dGdq(:,i) = (Gfun(qp) - Gfun(qm)) / (2*h);
end

A = [zeros(3), eye(3);
    -M_inv*dGdq, zeros(3)];
B = [zeros(3);
     M_inv];

%% --- 5. Modelo do MPC EM COORDENADAS DE DESVIO, com C = eye(6) ---
C_mpc = eye(6);
D_mpc = zeros(6,3);

plant_c = ss(A, B, C_mpc, D_mpc);

Ts = 0.02;
plant_d = c2d(plant_c, Ts);

Np = 30;
Nc = 8;

mpcobj = mpc(plant_d, Ts, Np, Nc);

%% --- 6. Pesos ---
% Saidas: [delta_q1 delta_q2 delta_q3 delta_qd1 delta_qd2 delta_qd3]
mpcobj.Weights.OutputVariables = [50 50 50 2 2 2];
mpcobj.Weights.ManipulatedVariables = [0 0 0];
mpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1];

%% --- 7. Limites de torque (em coordenadas de DESVIO, deslocados por u_star) ---
tau_min = [-5; -5; -5];
tau_max = [ 5;  5;  5];

for i = 1:3
    mpcobj.MV(i).Min = tau_min(i) - u_star(i);
    mpcobj.MV(i).Max = tau_max(i) - u_star(i);
end

%% --- 8. Estados/saidas nominais (mantidos em zero, ja que tudo e desvio) ---
mpcobj.Model.Nominal.X = zeros(6,1);
mpcobj.Model.Nominal.U = zeros(3,1);
mpcobj.Model.Nominal.Y = zeros(6,1);

assignin('base','mpcobj',mpcobj);

fprintf('\n=== Inicializacao concluida. Pronto para abrir o Simulink. ===\n');
fprintf('q_star = [%.1f, %.1f, %.1f] graus\n', rad2deg(q_star));
fprintf('u_star = [%.4f, %.4f, %.4f] N.m\n', u_star);
fprintf('MV limits (desvio): [%.4f,%.4f] [%.4f,%.4f] [%.4f,%.4f]\n', ...
    [tau_min-u_star, tau_max-u_star]');