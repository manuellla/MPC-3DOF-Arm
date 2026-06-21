function du0 = rrr_linearmpc_block(in)
% RRR_LINEARMPC_BLOCK  Usa a classe LinearMPC (QP esparso, quadprog) como
% alternativa ao bloco custom denso, pra comparação.
%
% Mesmo contrato de entrada/saída do bloco original:
%   in(1:6)  = dx0   = X - x_star
%   in(7:12) = dXref = Xref - x_star
%   du0 (3x1) -- soma com u_star FORA do bloco.

persistent mpcObj N Nx isInit

if isempty(isInit)
    [mpcObj, N, Nx] = local_init();
    isInit = true;
end

dx0   = in(1:6);
dXref = in(7:12);

refTraj = repmat(dXref, 1, N+1);   % Nx x (N+1): mesma referência ao longo do horizonte + estado terminal

[xout, ~] = mpcObj.solve(dx0, refTraj);
[u0, ~]   = mpcObj.getOutput(xout);

du0 = u0;

end


function [mpcObj, N, Nx] = local_init()

%% --- Robô (idêntico ao bloco original) ---
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
robot = SerialLink(L, 'name', 'RRR-LinearMPC', 'gravity', [0;0;-g]);

Gfun   = @(q) robot.gravload(q')';
u_star = Gfun(q_star);

M_star = robot.inertia(q_star');
M_inv  = inv(M_star);

dGdq = zeros(3,3); h = 1e-6;
for i = 1:3
    qp = q_star; qm = q_star;
    qp(i) = qp(i) + h; qm(i) = qm(i) - h;
    dGdq(:,i) = (Gfun(qp) - Gfun(qm)) / (2*h);
end

A = [zeros(3),       eye(3);
    -M_inv*dGdq,      zeros(3)];
B = [zeros(3); M_inv];

Ts = 0.02;
sysd = c2d(ss(A,B,eye(6),zeros(6,3)), Ts);
Ad = sysd.A; Bd = sysd.B;

%% --- Pesos (Q/Qn iguais ao original; SEM penalidade de taxa -- limitação da classe) ---
Nx = 6; Nu = 3;
N  = 30;   % horizonte (equivalente ao Np do bloco original)

Q  = diag([50 50 50 2 2 2]);
Qn = Q;                          % peso terminal -- pode aumentar se quiser mais "puxão" no fim do horizonte
R  = diag([1e-6 1e-6 1e-6]);     % ~0, só regularização numérica (ver nota abaixo)

%% --- Bounds ---
% Estado: sem limite físico real aqui (a planta linearizada não impõe
% limite de posição/velocidade) -- usa valores bem largos.
stateBounds = repmat([-1e3, 1e3], Nx, 1);

% Controle: igual ao bloco original, em forma de DESVIO
tau_min = [-1.5; -1.5; -1.5];
tau_max = [ 1.5;  1.5;  1.5];
u_min = tau_min - u_star;
u_max = tau_max - u_star;
controlBounds = [u_min, u_max];

mpcObj = LinearMPC(Ad, Bd, Q, Qn, R, stateBounds, controlBounds, N, 'Solver', 'quadprog');

end