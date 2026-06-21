%% plot_RRR_mpc_results.m
% Plots logged joint angles vs desired, and animates the resulting
% trajectory using Peter Corke's Robotics Toolbox.
%
% Assumes the Simulink model was run and produced a Simulink.SimulationOutput
% object called "out" in the base workspace, containing:
%   out.q_log     -- actual joint angles (3 columns: q1 q2 q3)
%   out.qdes_log  -- desired joint angles (3 columns: q1 q2 q3)
%   out.tout      -- time vector
%
% Run setup_RRR_mpc.m at least once before this (or have q_star/robot
% params available) is NOT required here -- this script rebuilds the
% robot model itself so it's self-contained.

clear q t qdes  % avoid stale variables from previous runs

%% --- Extract data robustly from the SimulationOutput object ---
% Depending on "Save format" chosen in the To Workspace blocks, the
% logged signal could come back as a plain array, a timeseries object,
% or wrapped in a "Structure With Time". Handle all three.

q    = extract_logged_signal(out.q_log);
qdes = extract_logged_signal(out.qdes_log);

% Time vector: prefer out.tout if present, else timeseries' own time
if isprop(out, 'tout') || isfield(out, 'tout')
    t = out.tout;
else
    t = (0:size(q,1)-1)';  % fallback: sample index, not real time
end

% Sanity check on shapes
if size(q,2) ~= 3 || size(qdes,2) ~= 3
    error(['Expected q_log and qdes_log to have 3 columns (one per joint). ' ...
           'Got size(q)=%s, size(qdes)=%s. Check To Workspace block wiring.'], ...
           mat2str(size(q)), mat2str(size(qdes)));
end

%% --- Plot joint angles: actual vs desired ---
figure('Name', 'Joint Angles: Actual vs Desired', 'Color', 'w');
jointNames = {'q_1', 'q_2', 'q_3'};
for i = 1:3
    subplot(3,1,i);
    plot(t, q(:,i), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, qdes(:,i), 'r--', 'LineWidth', 1.5);
    grid on;
    ylabel([jointNames{i} ' (rad)']);
    if i == 1
        legend('Actual', 'Desired', 'Location', 'best');
        title('Joint Angles: Actual vs Desired');
    end
    if i == 3
        xlabel('Time (s)');
    end
end

%% --- Rebuild the robot model (same parameters as setup_RRR_mpc.m) ---
g  = 9.81;
d1 = 0.0919;
a2 = 0.2385;
a3 = 0.235;
m1 = 0.20;
m2 = 0.40;
m3 = 0.30;

L(1) = Link('revolute', 'd', d1, 'a', 0,  'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0,  'a', a3, 'alpha', 0);

L(1).m = m1; L(1).r = [0, 0, -d1/2];
L(1).I = diag([m1*d1^2/12, m1*d1^2/12, 0]);

L(2).m = m2; L(2).r = [-a2/2, 0, 0];
L(2).I = diag([0, m2*a2^2/12, m2*a2^2/12]);

L(3).m = m3; L(3).r = [-a3/2, 0, 0];
L(3).I = diag([0, m3*a3^2/12, m3*a3^2/12]);

robot = SerialLink(L, ...
    'name', 'RRR-AX12A+garra', ...
    'gravity', [0; 0; -g]);

%% --- Animate the logged trajectory ---
% robot.plot expects an Nx3 matrix of joint angles (radians), one row
% per frame. We downsample if the log is very dense, so the animation
% doesn't take forever to play.

maxFrames = 200;  % cap animation frames for speed
N = size(q,1);
if N > maxFrames
    idx = round(linspace(1, N, maxFrames));
else
    idx = 1:N;
end

figure('Name', 'RRR Arm Trajectory', 'Color', 'w');
robot.plot(q(idx,:), 'trail', {'r', 'LineWidth', 2});

%% --- Helper function ---
function sig = extract_logged_signal(raw)
% Normalizes a logged Simulink signal (from a To Workspace block) into
% a plain [N x M] double array, regardless of which "Save format" was
% used (Array, timeseries, or Structure With Time).

    if isa(raw, 'timeseries')
        sig = raw.Data;
    elseif isstruct(raw) && isfield(raw, 'signals')
        % "Structure With Time" format
        sig = raw.signals.values;
    elseif isa(raw, 'double') || isa(raw, 'single')
        sig = double(raw);
    else
        error(['Unrecognized logged signal format (class: %s). ' ...
               'Expected double array, timeseries, or "Structure With Time".'], class(raw));
    end

    % Ensure time runs down rows, channels across columns (N x 3)
    if size(sig,2) > size(sig,1) && size(sig,1) <= 3
        sig = sig';
    end
end