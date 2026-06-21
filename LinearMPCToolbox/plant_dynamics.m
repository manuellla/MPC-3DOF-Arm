function xdot = plant_dynamics(u)
% u(1:6) = x = [q1 q2 q3 dq1 dq2 dq3]' (ABSOLUTO)
% u(7:9) = tau = [tau1 tau2 tau3]' (ABSOLUTO)

persistent robot_local
if isempty(robot_local)
    g  = 9.81;
    d1 = 0.0919;
    a2 = 0.2385;
    a3 = 0.235;
    m1 = 0.20; m2 = 0.40; m3 = 0.30;

    L(1) = Link('revolute', 'd', d1, 'a', 0,  'alpha', pi/2);
    L(2) = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0);
    L(3) = Link('revolute', 'd', 0,  'a', a3, 'alpha', 0);
    L(1).m = m1; L(1).r = [0, 0, -d1/2];   L(1).I = diag([m1*d1^2/12, m1*d1^2/12, 0]);
    L(2).m = m2; L(2).r = [-a2/2, 0, 0];   L(2).I = diag([0, m2*a2^2/12, m2*a2^2/12]);
    L(3).m = m3; L(3).r = [-a3/2, 0, 0];   L(3).I = diag([0, m3*a3^2/12, m3*a3^2/12]);

    robot_local = SerialLink(L, 'name', 'RRR-AX12A+garra', 'gravity', [0; 0; -g]);
end

x   = u(1:6);
tau = u(7:9);
q   = x(1:3);
dq  = x(4:6);

M = robot_local.inertia(q');
C = robot_local.coriolis(q', dq');
G = -robot_local.gravload(q')';

ddq = M \ (tau - C*dq - G);
xdot = [dq; ddq];
end