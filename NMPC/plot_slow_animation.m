%% ========================================================================
%  ANIMACAO LENTA COM TRACO DO EFETUADOR FINAL (reproducao ao vivo)
%  Rodar DEPOIS da simulacao Simulink (variavel 'out' no workspace)
%  Script independente - nao precisa rodar plot_results.m antes
% ========================================================================

%% --- 1. Extrair dados logados ---
t       = out.tout_log;
q_real  = out.q_log;

%% --- 2. Reconstroi o robot, caso nao esteja no workspace ---
if ~exist('robot','var')
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
end

%% --- 3. Slow-motion via interpolacao ---
% Em vez de pegar menos amostras (o que deixaria a animacao mais "pulada"),
% interpolamos MAIS pontos entre as amostras originais, deixando o
% movimento mais suave e, junto com um fps baixo, visivelmente mais lento.

interp_factor = 6;     % quantos frames interpolados entre cada par de amostras originais
playback_fps  = 20;    % fps de reproducao (mais baixo = mais lento)

N = numel(t);
t_fine = linspace(t(1), t(end), (N-1)*interp_factor + 1)';

q_fine = zeros(numel(t_fine), 3);
for i = 1:3
    q_fine(:,i) = interp1(t, q_real(:,i), t_fine, 'pchip');  % pchip evita overshoot artificial
end

fprintf('Trajetoria original: %d amostras -> Trajetoria interpolada: %d frames\n', N, numel(t_fine));

%% --- 4. Trajetoria cartesiana do efetuador (para o traco) ---
N_fine = size(q_fine, 1);
ee_traj = zeros(N_fine, 3);

for k = 1:N_fine
    Tk = robot.fkine(q_fine(k,:));
    ee_traj(k,:) = Tk.t';   % .t e 3x1 para uma unica pose -> transpoe para 1x3
end

%% --- 5. Animacao quadro a quadro, com traco acumulado do efetuador ---
fig = figure('Name','Animacao Lenta - Traco do Efetuador','Position',[250 250 900 700]);

% Limites do espaco de trabalho (ajuste se necessario para o seu alcance)
ws = [-0.55 0.55 -0.55 0.55 -0.05 0.55];

for k = 1:numel(t_fine)
    clf(fig)

    % Desenha o braco na configuracao atual
    robot.plot(q_fine(k,:), ...
        'workspace', ws, ...
        'jointdiam', 1.5, 'arrow', 'nobase', 'noname', 'nowrist');

    hold on

    % Traco acumulado do efetuador ate o frame atual
    plot3(ee_traj(1:k,1), ee_traj(1:k,2), ee_traj(1:k,3), ...
        'r-', 'LineWidth', 2);

    % Marca o ponto inicial (fixo) e a posicao atual do efetuador
    plot3(ee_traj(1,1), ee_traj(1,2), ee_traj(1,3), ...
        'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot3(ee_traj(k,1), ee_traj(k,2), ee_traj(k,3), ...
        'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r');

    title(sprintf('t = %.2f s', t_fine(k)))
    drawnow

    pause(1/playback_fps);   % controla a velocidade de reproducao ao vivo
end

fprintf('\n=== Animacao concluida. ===\n');