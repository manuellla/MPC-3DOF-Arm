function du0 = rrr_mpc_relin(in)
% RRR_CUSTOM_MPC_BLOCK  Linear MPC com RE-LINEARIZAÇÃO DINÂMICA em torno
% da referência atual, em vez de um único ponto fixo.
%
% Contrato de entrada/saída IDÊNTICO ao original (nada muda no Simulink):
%   in(1:6)  = dx0   = X - x_star
%   in(7:12) = dXref = Xref - x_star
%   du0 (3x1) -- soma com u_star (o ORIGINAL, fixo em q_star) FORA do bloco:
%       U = du0 + u_star
%
% POR DENTRO: quando a referência (q_ref) muda, o bloco relineariza Ad,
% Bd, dGdq, H, restrições em torno de q_ref (não mais de q_star fixo).
% A conversão de volta pro referencial original (x_star/u_star) é feita
% automaticamente, então o resto do Simulink nunca precisa saber disso.
%
% ATENÇÃO DE PERFORMANCE: a relinearização (Ad^k pra k=1..Np, Hessiana,
% etc.) só roda quando a referência muda de fato (comparada com
% tolerância). Se sua referência for um SETPOINT que muda raramente, isso
% é ótimo. Se for uma trajetória que muda TODO sample, isso vai rebuildar
% a cada passo e pode ficar lento/inviável em tempo real -- nesse caso
% me avisa que a estratégia precisa ser outra.

persistent robot mpcData con quadprogOpts q_lin u_lin u_star_orig x_star isInit

if isempty(isInit)
    x_star = [0; pi/4; -pi/6; 0; 0; 0];   % MESMO ponto original, fixo
    robot  = local_build_robot();

    q_star      = x_star(1:3);
    u_star_orig = local_gravity(robot, q_star);   % torque de equilíbrio ORIGINAL

    [mpcData, con] = local_build_at_point(robot, q_star);
    quadprogOpts = optimoptions('quadprog', 'Display', 'off');

    q_lin = q_star;          % ponto de linearização ATUAL (começa = q_star)
    u_lin = u_star_orig;     % torque de equilíbrio no ponto de linearização ATUAL

    isInit = true;
end

dx0   = in(1:6);
dXref = in(7:12);

%% --- Reconstrói sinais absolutos a partir do referencial fixo x_star ---
X    = x_star + dx0;
Xref = x_star + dXref;
q_ref = Xref(1:3);

%% --- Relineariza SE a referência mudou (tolerância evita rebuild por ruído numérico) ---
tol = 1e-4;   % rad (~0.006 deg) -- ajuste se necessário
if norm(q_ref - q_lin) > tol
    q_lin = q_ref;
    u_lin = local_gravity(robot, q_lin);
    [mpcData, con] = local_build_at_point(robot, q_lin);
end

%% --- Converte estado/referência pro referencial LOCAL (em torno de q_lin) ---
x_loc    = X    - [q_lin; zeros(3,1)];
xref_loc = Xref - [q_lin; zeros(3,1)];   % ~0 em posição (já que q_ref==q_lin); pode ser != 0 em velocidade se Xref tiver dqref != 0

%% --- Resolve o MPC no referencial local ---
du_loc = local_mpc_step(mpcData, con, x_loc, xref_loc, quadprogOpts);

%% --- Converte a saída DE VOLTA pro referencial original (x_star/u_star) ---
% U_absoluto = u_lin + du_loc
% Mas o Sum block externo faz: U = du0 + u_star_orig
% Logo:  du0 = U_absoluto - u_star_orig = (u_lin - u_star_orig) + du_loc
du0 = (u_lin - u_star_orig) + du_loc;

end


%% ===================================================================
function robot = local_build_robot()
g  = 9.81; d1 = 0.0919; a2 = 0.2385; a3 = 0.235;
m1 = 0.20; m2 = 0.40; m3 = 0.30;

L(1) = Link('revolute', 'd', d1, 'a', 0,  'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0,  'a', a3, 'alpha', 0);
L(1).m = m1; L(1).r = [0, 0, -d1/2];
L(1).I = diag([m1*d1^2/12, m1*d1^2/12, 0]);
L(2).m = m2; L(2).r = [-a2/2, 0, 0];
L(2).I = diag([0, m2*a2^2/12, m2*a2^2/12]);
L(3).m = m3; L(3).r = [-a3/2, 0, 0];
L(3).I = diag([0, m3*a3^2/12, m3*a3^2/12]);

robot = SerialLink(L, 'name', 'RRR-AX12A+garra', 'gravity', [0; 0; -g]);
end


%% ===================================================================
function G = local_gravity(robot, q)
% Convenção corrigida -- bate com rrr_nonlinear_xdot.m
G = -robot.gravload(q')';
end


%% ===================================================================
function [mpcData, con] = local_build_at_point(robot, q_lin)
% Reconstrói TUDO (linearização, discretização, Hessiana, restrições)
% em torno do ponto q_lin (em vez de um q_star fixo).

Gfun = @(q) local_gravity(robot, q);

%% --- Linearização (Jacobiano numérico da gravidade) em q_lin ---
M_lin = robot.inertia(q_lin');
M_inv = inv(M_lin);

dGdq = zeros(3,3);
h = 1e-6;
for i = 1:3
    qp = q_lin; qm = q_lin;
    qp(i) = qp(i) + h;
    qm(i) = qm(i) - h;
    dGdq(:,i) = (Gfun(qp) - Gfun(qm)) / (2*h);
end

A = [zeros(3),       eye(3);
    -M_inv*dGdq,      zeros(3)];
B = [zeros(3);
     M_inv];

%% --- Discretização ---
Ts = 0.02;
sysd = c2d(ss(A, B, eye(6), zeros(6,3)), Ts);
Ad = sysd.A;
Bd = sysd.B;

%% --- Pesos (idênticos ao original) ---
Np = 30;
Nc = 8;
Q     = diag([50 50 50 2 2 2]);
Rabs  = diag([0 0 0]);
Rrate = diag([0.1 0.1 0.1]);
P     = Q;

mpcData = local_build_dense_mpc(Ad, Bd, Q, Rabs, Rrate, P, Np, Nc);

%% --- Restrições (em torno do u_lin DESSE ponto) ---
u_lin_local = Gfun(q_lin);
tau_min = [-1.5; -1.5; -1.5];
tau_max = [ 1.5;  1.5;  1.5];
u_min = tau_min - u_lin_local;
u_max = tau_max - u_lin_local;

du_rate_max = [0.5; 0.5; 0.5];
du_rate_min = -du_rate_max;

con = local_build_constraints(mpcData, du_rate_min, du_rate_max, u_min, u_max);

end


%% ===================================================================
function mpcData = local_build_dense_mpc(Ad, Bd, Q, Rabs, Rrate, P, Np, Nc)

n = size(Ad,1);
m = size(Bd,2);

M_blk = zeros(m*Np, m*Nc);
for k = 1:Np
    if k <= Nc, colBlock = k; else, colBlock = Nc; end
    rows = (k-1)*m + (1:m);
    cols = (colBlock-1)*m + (1:m);
    M_blk(rows, cols) = eye(m);
end

Sx_full = zeros(n*Np, n);
Su_full = zeros(n*Np, m*Np);

Apow = eye(n);
for k = 1:Np
    Apow = Apow * Ad;
    Sx_full((k-1)*n+(1:n), :) = Apow;
end

for k = 1:Np
    for j = 1:k
        Apow_kj = Ad^(k-j);
        rows = (k-1)*n + (1:n);
        cols = (j-1)*m + (1:m);
        Su_full(rows, cols) = Apow_kj * Bd;
    end
end

Sx = Sx_full;
Su = Su_full * M_blk;

Qbar = kron(eye(Np-1), Q);
Qbar = blkdiag(Qbar, P);
Rabs_bar = kron(eye(Nc), Rabs);

Drate_full = zeros(m*Nc, m*Nc);
for k = 1:Nc
    rows = (k-1)*m + (1:m);
    Drate_full(rows, rows) = eye(m);
    if k > 1
        cols_km1 = (k-2)*m + (1:m);
        Drate_full(rows, cols_km1) = -eye(m);
    end
end
Rrate_bar_weight = kron(eye(Nc), Rrate);
RateCostMatrix = Drate_full' * Rrate_bar_weight * Drate_full;

H = Su' * Qbar * Su + Rabs_bar + RateCostMatrix;
H = (H + H')/2;

mpcData.Sx = Sx;
mpcData.Su = Su;
mpcData.Qbar = Qbar;
mpcData.M_blk = M_blk;
mpcData.H = H;
mpcData.n = n; mpcData.m = m; mpcData.Np = Np; mpcData.Nc = Nc;

end


%% ===================================================================
function con = local_build_constraints(mpcData, du_min, du_max, u_min, u_max)

m = mpcData.m; Nc = mpcData.Nc;
nU = m*Nc;

Ain = [];
bin = [];

if ~isempty(u_min) && ~isempty(u_max)
    I_nU = eye(nU);
    ub_rep = repmat(u_max, Nc, 1);
    lb_rep = repmat(u_min, Nc, 1);
    Ain = [Ain;  I_nU; -I_nU];
    bin = [bin;  ub_rep; -lb_rep];
end

if ~isempty(du_min) && ~isempty(du_max)
    Drate = zeros(Nc*m, nU);
    for k = 1:Nc
        rows = (k-1)*m + (1:m);
        cols_k = (k-1)*m + (1:m);
        Drate(rows, cols_k) = eye(m);
        if k > 1
            cols_km1 = (k-2)*m + (1:m);
            Drate(rows, cols_km1) = -eye(m);
        end
    end
    ub_rate = repmat(du_max, Nc, 1);
    lb_rate = repmat(du_min, Nc, 1);
    Ain = [Ain;  Drate; -Drate];
    bin = [bin;  ub_rate; -lb_rate];
end

con.Ain = Ain;
con.bin = bin;

end


%% ===================================================================
function du0 = local_mpc_step(mpcData, con, dx0, dXref, quadprogOpts)

Np = mpcData.Np;
m  = mpcData.m;
Nc = mpcData.Nc;

Xref_stack = repmat(dXref, Np, 1);

track_err0 = mpcData.Sx*dx0 - Xref_stack;
f = mpcData.Su' * mpcData.Qbar * track_err0;

[dU_sol, ~, exitflag] = quadprog(mpcData.H, f, con.Ain, con.bin, ...
                                   [], [], [], [], [], quadprogOpts);

if exitflag ~= 1
    warning('rrr_custom_mpc_block:quadprogFailed', ...
        'quadprog exitflag=%d at this step; applying du=0 fallback.', exitflag);
    dU_sol = zeros(m*Nc, 1);
end

du0 = dU_sol(1:m);

end