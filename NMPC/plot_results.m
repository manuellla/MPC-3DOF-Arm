%% ========================================================================
%  POS-PROCESSAMENTO: Plots de rastreamento + animacao do braco (RTB)
%  Rodar DEPOIS da simulacao Simulink (variavel 'out' no workspace)
% ========================================================================

%% --- 1. Extrair dados logados ---
% out.q_log, out.u e out.qdes_log sao arrays 'double' diretos (formato
% "Array" no To Workspace), NAO sao objetos timeseries - por isso nao tem
% .Time/.Data. O vetor de tempo vem separadamente de out.tout_log.
t       = out.tout_log;            % N x 1 - vetor de tempo (Clock logado)
q_real  = out.q_log;                % N x 3 - ja e a matriz direta

% q_des esta como 1x3xN (uma "pagina" 1x3 por amostra) - precisa remodelar
qdes_raw = out.qdes_log;            % 1 x 3 x N
q_des = squeeze(qdes_raw)';         % squeeze -> 3 x N, depois transpoe -> N x 3

u_log = out.u;                      % N x 3 - ja e a matriz direta

% Checagem rapida de consistencia
fprintf('size(q_real) = %s\n', mat2str(size(q_real)));
fprintf('size(q_des)  = %s\n', mat2str(size(q_des)));
fprintf('size(u_log)  = %s\n', mat2str(size(u_log)));

%% --- Pasta de saida para as figuras ---
results_dir = fullfile(pwd, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
fprintf('Figuras serao salvas em: %s\n', results_dir);

%% --- 2. Plot: Q real vs Q desejado (por junta) ---
figure('Name','Rastreamento de Posicao Articular','Position',[100 100 900 700]);

labels = {'q_1 (Base)', 'q_2 (Ombro)', 'q_3 (Cotovelo)'};

for i = 1:3
    subplot(3,1,i)
    plot(t, rad2deg(q_real(:,i)), 'b-', 'LineWidth', 1.5); hold on
    plot(t, rad2deg(q_des(:,i)),  'r--', 'LineWidth', 1.5);
    grid on
    ylabel('Angulo (graus)')
    title(labels{i})
    legend('q real','q desejado','Location','best')
    if i == 3
        xlabel('Tempo (s)')
    end
end
sgtitle('Rastreamento de Trajetoria - NMPC')

exportgraphics(gcf, fullfile(results_dir, 'rastreamento_juntas.png'), 'Resolution', 200);

%% --- 3. Plot: Torques de controle ---
figure('Name','Sinais de Controle (Torques)','Position',[150 150 900 500]);

tau_labels = {'\tau_1 (N.m)', '\tau_2 (N.m)', '\tau_3 (N.m)'};
tau_max = 1.5;   % limite do AX-12A (Anexo A) - linha de referencia no grafico

for i = 1:3
    subplot(3,1,i)
    plot(t, u_log(:,i), 'k-', 'LineWidth', 1.3); hold on
    yline( tau_max, 'r--', 'LineWidth', 1, 'Label','Limite superior');
    yline(-tau_max, 'r--', 'LineWidth', 1, 'Label','Limite inferior');
    grid on
    ylabel(tau_labels{i})
    if i == 3
        xlabel('Tempo (s)')
    end
end
sgtitle('Torques Aplicados pelo Controlador NMPC')

exportgraphics(gcf, fullfile(results_dir, 'torques_controle.png'), 'Resolution', 200);

%% --- 4. Plot: Erro de rastreamento ---
figure('Name','Erro de Rastreamento','Position',[200 200 900 500]);
erro = rad2deg(q_des - q_real);

plot(t, erro(:,1), 'LineWidth', 1.3); hold on
plot(t, erro(:,2), 'LineWidth', 1.3);
plot(t, erro(:,3), 'LineWidth', 1.3);
yline(0, 'k:');
grid on
xlabel('Tempo (s)')
ylabel('Erro (graus)')
legend('erro q_1','erro q_2','erro q_3','Location','best')
title('Erro de Rastreamento por Junta')

exportgraphics(gcf, fullfile(results_dir, 'erro_rastreamento.png'), 'Resolution', 200);

%% --- 5. Animacao do braco usando o Robotics Toolbox ---
% Reconstroi o robot, caso nao esteja mais no workspace
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

% Subamostra a trajetoria para a animacao nao ficar excessivamente longa/pesada
n_frames = 200;
idx = round(linspace(1, numel(t), min(n_frames, numel(t))));

figure('Name','Animacao do Manipulador','Position',[250 250 800 600]);
robot.plot(q_real(idx,:), ...
    'workspace', [-0.6 0.6 -0.6 0.6 -0.05 0.6], ...
    'jointdiam', 1.5, 'arrow', 'nobase', 'trail', 'b-', 'fps', 15);

% Salva a ultima postura (final da animacao, com o rastro completo) como imagem estatica
exportgraphics(gcf, fullfile(results_dir, 'animacao_postura_final.png'), 'Resolution', 200);

% Gera tambem uma figura separada so com a postura inicial, para comparacao
figure('Name','Postura Inicial','Position',[300 300 800 600]);
robot.plot(q_real(1,:), ...
    'workspace', [-0.6 0.6 -0.6 0.6 -0.05 0.6], ...
    'jointdiam', 1.5, 'arrow', 'nobase');
title('Postura Inicial do Manipulador')
exportgraphics(gcf, fullfile(results_dir, 'postura_inicial.png'), 'Resolution', 200);

fprintf('\n=== Pos-processamento concluido. Figuras salvas em: %s ===\n', results_dir);