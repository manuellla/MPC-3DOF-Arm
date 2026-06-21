clear; clc; close all;

%% DEFINIÇÃO DO ROBÔ
g = 9.81;
d1 = 0.0919;
a2 = (0.1720 + 0.065);
a3 = 0.235;
m1 = 0.20; m2 = 0.40; m3 = 0.30;

L(1) = Link('revolute', 'd', d1, 'a', 0,  'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0   );
L(3) = Link('revolute', 'd', 0,  'a', a3, 'alpha', 0   );

L(1).m = m1; L(1).r = [0, 0, -d1/2]; L(1).I = diag([m1*d1^2/12, m1*d1^2/12, 0]);
L(2).m = m2; L(2).r = [-a2/2, 0, 0]; L(2).I = diag([0, m2*a2^2/12, m2*a2^2/12]);
L(3).m = m3; L(3).r = [-a3/2, 0, 0]; L(3).I = diag([0, m3*a3^2/12, m3*a3^2/12]);

robot = SerialLink(L, 'name', 'RRR', 'gravity', [0; 0; -g]);

%% PONTO DE EQUILÍBRIO E TORQUE
q_star  = [0; pi/4; -pi/6];
u_star  = -robot.gravload(q_star')';

%% VALIDAÇÃO: dinâmica não-linear com u* e q*
x0 = [q_star; zeros(3,1)];
t_span = [0 5];

odefun = @(t, x) dynamics_nonlinear(t, x, robot, u_star);
[t, x] = ode45(odefun, t_span, x0);

%% Plot
figure('Name', 'Validacao Equilibrio');
subplot(2,1,1);
plot(t, rad2deg(x(:,1:3)));
xlabel('Tempo (s)'); ylabel('Posição (graus)');
legend('\theta_1', '\theta_2', '\theta_3');
title('Posições das Juntas com u* aplicado');
yline(rad2deg(q_star(1)), '--', 'Color', '#0072BD');
yline(rad2deg(q_star(2)), '--', 'Color', '#D95319');
yline(rad2deg(q_star(3)), '--', 'Color', '#EDB120');
grid on;

subplot(2,1,2);
plot(t, x(:,4:6));
xlabel('Tempo (s)'); ylabel('Velocidade (rad/s)');
legend('dtheta1/dt', 'dtheta2/dt', 'dtheta3/dt');
title('Velocidades das Juntas (devem ser zero)');
grid on;

%% Função
function dxdt = dynamics_nonlinear(~, x, robot, u)
    q  = x(1:3);
    dq = x(4:6);
    M  = robot.inertia(q');
    C  = robot.coriolis(q', dq');
    G  = -robot.gravload(q')';
    ddq = M \ (u - C*dq - G);
    dxdt = [dq; ddq];
end