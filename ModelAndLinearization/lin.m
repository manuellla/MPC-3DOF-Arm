clear; clc; close all;

g = 9.81;

% Geometria dos elos (metros)
d1 = 0.0919;  
a2 = (0.1720 + 0.065); 
a3 = 0.235;

m1 = 0.20;
m2 = 0.40;
m3 = 0.30;

q_star = [0; pi/4;-pi/6]; % Ponto de equilíbrio q* (radianos)

L(1) = Link('revolute', 'd', d1, 'a', 0,  'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0   );
L(3) = Link('revolute', 'd', 0,  'a', a3, 'alpha', 0   );

% Link 1: barra vertical de comprimento d1
L(1).m = m1;
L(1).r = [0, 0, -d1/2];
L(1).I = diag([m1*d1^2/12, m1*d1^2/12, 0]);

% Link 2: barra ao longo de x2, comprimento a2
L(2).m = m2;
L(2).r = [-a2/2, 0, 0];
L(2).I = diag([0, m2*a2^2/12, m2*a2^2/12]);

% Link 3: barra ao longo de x3, comprimento a3
L(3).m = m3;
L(3).r = [-a3/2, 0, 0];
L(3).I = diag([0, m3*a3^2/12, m3*a3^2/12]);

% gravity = [0;0;-g], (z0 aponta para cima).
% gravload retorna o torque que a gravidade CAUSA (negativo = braço tende a cair).
% Invertemos G para obter u* no sentido do relatório (torque para SUSTENTAR).
robot = SerialLink(L, 'name', 'RRR-AX12A+garra', 'gravity', [0; 0; -g]);

M_star = robot.inertia(q_star');
G_star = -robot.gravload(q_star')';    % inverte, torque para SUSTENTAR o braço
u_star = G_star;
M_inv  = inv(M_star);

% Jacobiano numérico de G(q) em q*
dGdq = zeros(3,3);
h    = 1e-7;
for i = 1:3
    qp        = q_star;  qp(i) = qp(i) + h;
    dGdq(:,i) = -(robot.gravload(qp') - robot.gravload(q_star'))' / h;
end

A = [ zeros(3),      eye(3)   ;
     -M_inv * dGdq,  zeros(3) ];

B = [ zeros(3) ;
      M_inv    ];

% resultados
fprintf('=== LINEARIZACAO DO MANIPULADOR RRR + GARRA ===\n\n')

fprintf('Equilibrio q* = [%.1f,  %.1f,  %.1f] graus\n\n', ...
        rad2deg(q_star(1)), rad2deg(q_star(2)), rad2deg(q_star(3)));

T_star = robot.fkine(q_star');
fprintf('Posicao da ponta da garra em q*:\n');
fprintf('  x = %.4f m,  y = %.4f m,  z = %.4f m\n\n', ...
        T_star.t(1), T_star.t(2), T_star.t(3));

fprintf('Torques de equilibrio u* = G(q*):\n');
fprintf('  tau1 = %+.4f N.m\n  tau2 = %+.4f N.m\n  tau3 = %+.4f N.m\n\n', u_star);

fprintf('Matriz M(q*):\n'); disp(M_star)
fprintf('Matriz A (6x6):\n'); disp(A)
fprintf('Matriz B (6x3):\n'); disp(B)

ev = eig(A);
fprintf('Autovalores de A:\n'); disp(ev)
if all(real(ev) < 1e-8)
    fprintf('-> Marginalmente estavel em malha aberta\n\n')
else
    fprintf('-> INSTAVEL em malha aberta\n\n')
end

Co = ctrb(A, B);
r  = rank(Co);
if r == size(A,1)
    fprintf('Controlabilidade: posto = %d/%d -> CONTROLAVEL\n\n', r, size(A,1));
else
    fprintf('Controlabilidade: posto = %d/%d -> NAO CONTROLAVEL\n\n', r, size(A,1));
end

figure('Name', 'Configuracao de Equilibrio', 'Position', [100 100 800 600]);
robot.plot(q_star', ...
    'workspace', [-0.6 0.6 -0.6 0.6 -0.05 0.6], ...
    'jointdiam', 1.5, 'arrow', 'nobase');
title(sprintf('q* = [0, 45, -30] graus   tau* = [%.3f, %.3f, %.3f] N.m', u_star), ...
      'FontSize', 11);