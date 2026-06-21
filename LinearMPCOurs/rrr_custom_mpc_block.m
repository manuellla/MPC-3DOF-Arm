function du0 = rrr_custom_mpc_block(in)
% RRR_CUSTOM_MPC_BLOCK  Self-contained linear MPC controller for Simulink.
% Everything -- robot model, linearization, dense QP matrices,
% constraints, and the quadprog solve -- lives in this ONE file as local
% subfunctions, so you can set breakpoints anywhere and step through the
% whole pipeline without jumping between files.
%
% Use inside an INTERPRETED MATLAB FUNCTION block (quadprog is not
% Coder-compatible, so this cannot be a plain MATLAB Function block).
%
% Input "in" is a 12x1 vector built by a Mux upstream:
%   in(1:6)  = dx0   = X - x_star      (current deviation state)
%   in(7:12) = dXref = Xref - x_star   (deviation reference, held over horizon)
%
% Output: du0 (3x1) -- the deviation torque move to apply THIS sample.
% Add u_star OUTSIDE this block (Sum block) to get absolute torque:
%     U = du0 + u_star
%
% All the expensive matrix-building happens ONCE on the first call via
% persistent caching, not every sample.

persistent mpcData con quadprogOpts isInit

if isempty(isInit)
    [mpcData, con, quadprogOpts] = local_init();
    isInit = true;
end

dx0   = in(1:6);
dXref = in(7:12);

du0 = local_mpc_step(mpcData, con, dx0, dXref, quadprogOpts);

end


%% ===================================================================
%  LOCAL SUBFUNCTION: local_init
%  Builds the robot, linearizes it, discretizes, builds dense QP
%  matrices and constraints. Runs ONCE (cached via persistent above).
%  =====================================================================
function [mpcData, con, quadprogOpts] = local_init()

%% --- Robot parameters ---
g  = 9.81;
d1 = 0.0919;
a2 = 0.2385;
a3 = 0.235;
m1 = 0.20;
m2 = 0.40;
m3 = 0.30;

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

robot = SerialLink(L, 'name', 'RRR-AX12A+garra', 'gravity', [0; 0; -g]);

%% --- Equilibrium torque ---
Gfun   = @(q) -robot.gravload(q')';
u_star = Gfun(q_star);   %#ok<NASGU> -- kept for reference/debugging visibility

%% --- Linearization (numeric Jacobian of gravity load) ---
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

A = [zeros(3),       eye(3);
    -M_inv*dGdq,      zeros(3)];
B = [zeros(3);
     M_inv];

%% --- Discretize (exact zero-order-hold, matches the MPC Toolbox version) ---
Ts = 0.02;
sysc = ss(A, B, eye(6), zeros(6,3));
sysd = c2d(sysc, Ts);
Ad = sysd.A;
Bd = sysd.B;

%% --- MPC tuning ---
% Rabs=0 matches toolbox Weights.ManipulatedVariables=[0 0 0] (no penalty
% on absolute deviation torque -- penalizing this fights against holding
% the nonzero steady-state torque needed at a new equilibrium, which was
% the root cause of the steady-state-error bug found earlier).
% Rrate matches toolbox Weights.ManipulatedVariablesRate=[0.1 0.1 0.1].
Np = 30;
Nc = 8;

Q     = diag([50 50 50 2 2 2]);
Rabs  = diag([0 0 0]);
Rrate = diag([0.1 0.1 0.1]);
P     = Q;

mpcData = local_build_dense_mpc(Ad, Bd, Q, Rabs, Rrate, P, Np, Nc);
mpcData.u_star = u_star;   % stash for debugging/inspection if needed

%% --- Constraints (deviation form) ---
tau_min = [-1.5; -1.5; -1.5];
tau_max = [ 1.5;  1.5;  1.5];
u_min = tau_min - u_star;
u_max = tau_max - u_star;

du_rate_max = [0.5; 0.5; 0.5];
du_rate_min = -du_rate_max;

con = local_build_constraints(mpcData, du_rate_min, du_rate_max, u_min, u_max);

quadprogOpts = optimoptions('quadprog', 'Display', 'off');

end


%% ===================================================================
%  LOCAL SUBFUNCTION: local_build_dense_mpc
%  Builds Sx, Su, and the QP Hessian H (offline, constant matrices).
%  =====================================================================
function mpcData = local_build_dense_mpc(Ad, Bd, Q, Rabs, Rrate, P, Np, Nc)

n = size(Ad,1);
m = size(Bd,2);

%% --- Blocking-move expansion matrix: U_full = M_blk * dU ---
M_blk = zeros(m*Np, m*Nc);
for k = 1:Np
    if k <= Nc
        colBlock = k;
    else
        colBlock = Nc;
    end
    rows = (k-1)*m + (1:m);
    cols = (colBlock-1)*m + (1:m);
    M_blk(rows, cols) = eye(m);
end

%% --- Prediction matrices over the full horizon ---
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

%% --- Weight matrices ---
Qbar = kron(eye(Np-1), Q);
Qbar = blkdiag(Qbar, P);
Rabs_bar = kron(eye(Nc), Rabs);

%% --- Rate-penalty matrix: penalizes (du_k - du_{k-1}), du_{-1}=0 ---
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

%% --- QP Hessian ---
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
%  LOCAL SUBFUNCTION: local_build_constraints
%  Builds Ain, bin for input bounds and input rate bounds.
%  =====================================================================
function con = local_build_constraints(mpcData, du_min, du_max, u_min, u_max)

m = mpcData.m; Nc = mpcData.Nc;
nU = m*Nc;

Ain = [];
bin = [];

%% --- Absolute deviation-input bounds ---
if ~isempty(u_min) && ~isempty(u_max)
    I_nU = eye(nU);
    ub_rep = repmat(u_max, Nc, 1);
    lb_rep = repmat(u_min, Nc, 1);
    Ain = [Ain;  I_nU; -I_nU];
    bin = [bin;  ub_rep; -lb_rep];
end

%% --- Input rate bounds (du_{-1} assumed 0) ---
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
%  LOCAL SUBFUNCTION: local_mpc_step
%  Runtime QP solve: builds the gradient from the current state/ref,
%  calls quadprog, returns the first move (receding horizon).
%  =====================================================================
function du0 = local_mpc_step(mpcData, con, dx0, dXref, quadprogOpts)

Np = mpcData.Np;
m  = mpcData.m;
Nc = mpcData.Nc;

%% --- Expand reference to full stacked horizon vector ---
Xref_stack = repmat(dXref, Np, 1);

%% --- QP gradient ---
track_err0 = mpcData.Sx*dx0 - Xref_stack;
f = mpcData.Su' * mpcData.Qbar * track_err0;

%% --- Solve QP ---
[dU_sol, ~, exitflag] = quadprog(mpcData.H, f, con.Ain, con.bin, ...
                                   [], [], [], [], [], quadprogOpts);

if exitflag ~= 1
    warning('rrr_custom_mpc_block:quadprogFailed', ...
        'quadprog exitflag=%d at this step; applying du=0 fallback.', exitflag);
    dU_sol = zeros(m*Nc, 1);
end

du0 = dU_sol(1:m);

end