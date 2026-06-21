clear; clc; close all;

% Parametros fisicos do manipulador
g  = 9.81;
d1 = 0.0919;
a2 = 0.1720 + 0.065;   % 0.2385
a3 = 0.235;

m1 = 0.20;
m2 = 0.40;
m3 = 0.30;

% Modelo SerialLink (RTB)
L(1) = Link('revolute', 'd', d1, 'a', 0,  'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0   );
L(3) = Link('revolute', 'd', 0,  'a', a3, 'alpha', 0   );

L(1).m = m1; L(1).r = [0, 0, -d1/2];   L(1).I = diag([m1*d1^2/12, m1*d1^2/12, 0]);
L(2).m = m2; L(2).r = [-a2/2, 0, 0];   L(2).I = diag([0, m2*a2^2/12, m2*a2^2/12]);
L(3).m = m3; L(3).r = [-a3/2, 0, 0];   L(3).I = diag([0, m3*a3^2/12, m3*a3^2/12]);

robot = SerialLink(L, 'name', 'RRR-AX12A+garra', 'gravity', [0; 0; -g]);
assignin('base','robot',robot);

% Condicao inicial da simulacao (NAO e ponto de linearizacao - NMPC nao precisa de um) 
q0  = [0; pi/4; -pi/6];   % postura inicial do braço (rad), so para iniciar a simulacao
qd0 = [0; 0; 0];

assignin('base','q0',q0);
assignin('base','qd0',qd0);

% Parametros do NMPC 
Ts = 0.02;
assignin('base','Ts',Ts);

nx = 6;   % estados: [q1 q2 q3 qd1 qd2 qd3]
ny = 3;   % saidas: q1 q2 q3
nu = 3;   % entradas: tau1 tau2 tau3

nlobj = nlmpc(nx, ny, nu);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 15;
nlobj.ControlHorizon = 3;

nlobj.Model.StateFcn = "arm_state";     % usa robot.accel() internamente
nlobj.Model.IsContinuousTime = true;
nlobj.Model.OutputFcn = @(x,u) x(1:3);

nlobj.Weights.OutputVariables = [1 1 1];
nlobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1];

% Limites de torque (ajustar conforme datasheet AX-12A, Anexo A)
for i = 1:3
    nlobj.MV(i).Min = -2;   % N.m
    nlobj.MV(i).Max =  2;   % N.m
end

x0_test = [q0; qd0];
u0_test = [0; 0; 0];   % qualquer torque valido apenas para testar as dimensoes/forma

validateFcns(nlobj, x0_test, u0_test);

assignin('base','nlobj',nlobj);