%% ========================================================================
%  SCRIPT DE INICIALIZACAO - MPC LINEAR
%  Rodar ANTES de abrir/rodar o modelo Simulink
% ========================================================================
clear; clc; close all;

%% --- 1. Parametros fisicos e SerialLink (RTB) ---
g  = 9.81;
d1 = 0.0919;
a2 = 0.1720 + 0.065;
a3 = 0.235;
m1 = 0.20; m2 = 0.40; m3 = 0.30;

L(1) = Link('revolute', 'd', d1, 'a', 0,  'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0   );
L(3) = Link('revolute', 'd', 0,  'a', a3, 'alpha', 0   );

L(1).m = m1; L(1).r = [0, 0, -d1/2];   L(1).I = diag([m1*d1^2/12, m1*d1^2/12, 0]);
L(2).m = m2; L(2).r = [-a2/2, 0, 0];   L(2).I = diag([0, m2*a2^2/12, m2*a2^2/12]);
L(3).m = m3; L(3).r = [-a3/2, 0, 0];   L(3).I = diag([0, m3*a3^2/12, m3*a3^2/12]);

robot = SerialLink(L, 'name', 'RRR-AX12A+garra', 'gravity', [0; 0; -g]);
assignin('base','robot',robot);

%% --- 2. Ponto de equilibrio q_star, x_star, u_star ---
q_star  = [0; pi/4; -pi/6];          % ponto de linearizacao (Eq. 2.59 do relatorio)
x_star  = [q_star; 0; 0; 0];          % estado completo no equilibrio (velocidade nula)

M_star  = robot.inertia(q_star');
G_star  = -robot.gravload(q_star')';  % torque para SUSTENTAR o braco
u_star  = G_star;

assignin('base','q_star',q_star);
assignin('base','x_star',x_star);
assignin('base','u_star',u_star);

%% --- 3. Matrizes A, B linearizadas (Jacobiano numerico de G(q), igual Anexo B) ---
M_inv = inv(M_star);
dGdq  = zeros(3,3);
h = 1e-7;
for i = 1:3
    qp = q_star; qp(i) = qp(i) + h;
    dGdq(:,i) = -(robot.gravload(qp') - robot.gravload(q_star'))' / h;
end

A = [zeros(3),      eye(3)   ;
    -M_inv*dGdq,    zeros(3)];
B = [zeros(3) ;
     M_inv    ];

C = [eye(3), zeros(3)];   % saida = q (Eq. 2.69) - SOMENTE posicao, sem velocidade
D = zeros(3,3);

assignin('base','A',A);
assignin('base','B',B);
assignin('base','C',C);
assignin('base','D',D);

%% --- 4. Objeto de planta para o MPC (sistema continuo, o mpc() discretiza internamente) ---
sys_lin = ss(A, B, C, D);
sys_lin.InputName  = {'tau1','tau2','tau3'};
sys_lin.OutputName = {'q1','q2','q3'};

%% --- 5. Parametros do MPC Linear ---
Ts = 0.02;                  % mesmo Ts usado no NMPC, por consistencia comparativa
p  = 15;                    % horizonte de predicao
m  = 3;                     % horizonte de controle

mpcobj = mpc(sys_lin, Ts, p, m);

% Pesos - penaliza erro de saida, suaviza variacao de controle
mpcobj.Weights.OutputVariables      = [1 1 1];
mpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1];

% IMPORTANTE: nao definir MV.Min/Max aqui, pois o MPC atua sobre delta_u,
% nao sobre o torque absoluto. O limite fisico (+-1.5 N.m) sera aplicado
% no Simulink via bloco Saturation, DEPOIS de somar u_star (ver Etapa 6).

assignin('base','mpcobj',mpcobj);

fprintf('\n=== Inicializacao concluida. Pronto para abrir o Simulink. ===\n');
fprintf('q_star = [%.1f, %.1f, %.1f] graus\n', rad2deg(q_star));
fprintf('u_star = [%.4f, %.4f, %.4f] N.m\n', u_star);