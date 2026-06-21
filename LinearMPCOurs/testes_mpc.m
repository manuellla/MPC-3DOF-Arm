% %% TESTE 4 (revisado) — Malha fechada pura MATLAB, sem Simulink
% clear rrr_custom_mpc_block
% 
% %% Reconstrói o modelo linear (igual ao local_init) só pra simular a planta
% g  = 9.81; d1 = 0.0919; a2 = 0.2385; a3 = 0.235;
% m1 = 0.20; m2 = 0.40; m3 = 0.30;
% q_star = [0; pi/4; -pi/6];
% 
% L(1) = Link('revolute', 'd', d1, 'a', 0,  'alpha', pi/2);
% L(2) = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0);
% L(3) = Link('revolute', 'd', 0,  'a', a3, 'alpha', 0);
% L(1).m = m1; L(1).r = [0, 0, -d1/2];
% L(1).I = diag([m1*d1^2/12, m1*d1^2/12, 0]);
% L(2).m = m2; L(2).r = [-a2/2, 0, 0];
% L(2).I = diag([0, m2*a2^2/12, m2*a2^2/12]);
% L(3).m = m3; L(3).r = [-a3/2, 0, 0];
% L(3).I = diag([0, m3*a3^2/12, m3*a3^2/12]);
% robot = SerialLink(L, 'name', 'RRR-test', 'gravity', [0;0;-g]);
% 
% Gfun = @(q) robot.gravload(q')';
% M_star = robot.inertia(q_star');
% M_inv = inv(M_star);
% 
% dGdq = zeros(3,3); h = 1e-6;
% for i = 1:3
%     qp = q_star; qm = q_star;
%     qp(i) = qp(i) + h; qm(i) = qm(i) - h;
%     dGdq(:,i) = (Gfun(qp) - Gfun(qm)) / (2*h);
% end
% 
% A = [zeros(3), eye(3); -M_inv*dGdq, zeros(3)];
% B = [zeros(3); M_inv];
% Ts = 0.02;
% sysd = c2d(ss(A,B,eye(6),zeros(6,3)), Ts);
% Ad = sysd.A; Bd = sysd.B;
% 
% %% Malha fechada: planta linear + bloco MPC custom (sem Simulink)
% N = 500;                              % 500 * 0.02s = 10s simulados
% dx = zeros(6,1);                      % começa exatamente em q_star
% dXref = [0; 0.10; -0.10; 0; 0; 0];    % referência fixa
% dx_hist = zeros(6, N);
% 
% for k = 1:N
%     in_k = [dx; dXref];
%     du0 = rrr_custom_mpc_block(in_k);  % mesmo bloco custom, persistent mantém init
%     dx = Ad*dx + Bd*du0;               % avança a planta LINEAR (não o robô não-linear)
%     dx_hist(:,k) = dx;
% end
% 
% fprintf('dx final (q2, q3): [%.4f, %.4f]\n', dx(2), dx(3));
% fprintf('dXref   (q2, q3): [%.4f, %.4f]\n', dXref(2), dXref(3));
% 
% figure;
% plot((1:N)*Ts, dx_hist(2,:), 'b', (1:N)*Ts, dx_hist(3,:), 'g'); hold on;
% yline(dXref(2), 'b--'); yline(dXref(3), 'g--');
% legend('q2 (dx)', 'q3 (dx)', 'q2 ref', 'q3 ref');
% xlabel('Tempo (s)'); ylabel('Desvio (rad)');
% title('Malha fechada: planta linear + MPC custom (sem Simulink)');

%% TESTE 6 — Malha fechada com planta NÃO-LINEAR real
clear; clc; close all
clear rrr_custom_mpc_block

%% --- Robô (igual ao local_init) ---
g  = 9.81; d1 = 0.0919; a2 = 0.2385; a3 = 0.235;
m1 = 0.20; m2 = 0.40; m3 = 0.30;
q_star = [0; pi/4; -pi/6];

L(1) = Link('revolute', 'd', d1, 'a', 0,  'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0,  'a', a3, 'alpha', 0);
L(1).m = m1; L(1).r = [0, 0, -d1/2];
L(1).I = diag([m1*d1^2/12, m1*d1^2/12, 0]);
L(2).m = m2; L(2).r = [-a2/2, 0, 0];
L(2).I = diag([0, m2*a2^2/12, m2*a2^2/12]);
L(3).m = m3; L(3).r = [-a3/2, 0, 0];
L(3).I = diag([0, m3*a3^2/12, m3*a3^2/12]);
robot = SerialLink(L, 'name', 'RRR-test', 'gravity', [0;0;-g]);

Gfun = @(q) robot.gravload(q')';

%% --- Condições do teste ---
u_star = Gfun(q_star);
dXref  = [0; 0.10; -0.10; 0; 0; 0];
Ts = 0.02;
N  = 1000;  % 20s simulados

q  = q_star;       % começa exatamente no ponto de equilíbrio
qd = zeros(3,1);
dx_hist = zeros(6,N);

for k = 1:N
    dx = [q - q_star; qd];
    in_k = [dx; dXref];
    du0 = rrr_custom_mpc_block(in_k);
    u = du0 + u_star;                  % torque absoluto real

    M_q = robot.inertia(q');
    G_q = Gfun(q);
    qdd = M_q \ (u - G_q);              % ignorando Coriolis/atrito por ora

    qd = qd + Ts*qdd;
    q  = q  + Ts*qd;

    dx_hist(:,k) = [q - q_star; qd];
end

fprintf('dx final (q2,q3) NAO-LINEAR: [%.4f, %.4f]\n', dx_hist(2,end), dx_hist(3,end));
fprintf('dXref (q2,q3):                [%.4f, %.4f]\n', dXref(2), dXref(3));

figure;
t = (1:N)*Ts;
plot(t, dx_hist(2,:), 'b', t, dx_hist(3,:), 'g'); hold on;
yline(dXref(2), 'b--'); yline(dXref(3), 'g--');
legend('q2 (dx)', 'q3 (dx)', 'q2 ref', 'q3 ref');
xlabel('Tempo (s)'); ylabel('Desvio (rad)');
title('Malha fechada: planta NAO-LINEAR + MPC custom (sem Simulink)');

clear; clc; close all
clear rrr_linearmpc_block

% ... (mesmo setup de robot, q_star, Gfun, M_star, dGdq, A, B, Ad, Bd do Teste 4) ...

dx0   = zeros(6,1);
dXref = [0; 0.10; -0.10; 0; 0; 0];
N_steps = 500;
dx_hist = zeros(6, N_steps);
dx = dx0;

for k = 1:N_steps
    in_k = [dx; dXref];
    du0 = rrr_linearmpc_block(in_k);
    dx = Ad*dx + Bd*du0;
    dx_hist(:,k) = dx;
end

fprintf('dx final (q2,q3) [LinearMPC class]: [%.4f, %.4f]\n', dx(2), dx(3));