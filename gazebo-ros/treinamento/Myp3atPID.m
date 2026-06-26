% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %
% % Experimento de seguimento de parede com o robô Pioneer3AT no CoppeliaSim
% % usando controlador PID e dois sensores ultrassônicos (45° e 90°).
% %
% % - Sensores:
% %     * 45°: detecção antecipada (frontal-esquerdo)
% %     * 90°: distância lateral à parede (principal para o controle)
% %
% % - Atuadores:
% %     * Roda direita: velocidade fixa
% %     * Roda esquerda: velocidade modulada pelo PID
% %
% % - Este script:
% %     * Pergunta qual algoritmo foi usado na sintonia (PSO / FLA / manual)
% %       e seleciona o conjunto de ganhos correspondente.
% %     * Pergunta o rótulo do cenário (ex.: '1', '2', '3', ...),
% %       usado no nome do arquivo de log (log_PSO_L.mat, etc.).
% %     * Roda a simulação por 2 minutos (tempo de robô) com Ts = 0.05 s.
% %     * Plota distâncias, erro e velocidade da roda esquerda.
% %     * Salva os dados em um arquivo .mat para análise posterior.
% %
% % Código base: Mario Andrés Pastrana Triana (Out-25)
% % Modificado e expandido por: Sérgio Cruz (Dez-25)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear;
clc;

%% =================== SELEÇÃO DO ALGORITMO E CENÁRIO =======================
disp('Selecione o conjunto de ganhos PID:');
disp('  1 - PSO');
disp('  2 - FLA');
disp('  3 - Manual (definir na mão)');
algChoice = input('Opção (1/2/3): ');

switch algChoice
    case 1
        algTag = 'PSO';
        % Ganhos do controlador PID - PSO
        Kp = 1.00000;
        Ki = 1.00000;
        Kd = 0.00115;
    case 2
        algTag = 'FLA';
        % Ganhos do controlador PID - FLA
        Kp = 0.63236;
        Ki = 0.95949;
        Kd = 0.09713;
    otherwise
        algTag = 'MANUAL';
        fprintf('Informe os ganhos manuais do PID:\n');
        Kp = input('  Kp = ');
        Ki = input('  Ki = ');
        Kd = input('  Kd = ');
end

cenarioTag = input('Informe o rótulo do cenário (ex.: 1, 2, 3, ..., 8): ','s');

fprintf('\nAlgoritmo: %s | Cenário: %s\n', algTag, cenarioTag);
fprintf('Gains PID: Kp=%.5f, Ki=%.5f, Kd=%.5f\n\n', Kp, Ki, Kd);

%% ============== INICIALIZAÇÃO GAZEBO =======================

% Inicializa o nó ROS 2 no MATLAB
node = ros2node("/matlab_pose_controller");

% Cria o publisher mirando no tópico mapeado pela ponte
pub = ros2publisher(node, "/model/pioneer3at/pose", "geometry_msgs/Pose");
msg = ros2message(pub);

% Configura a posição
msg.position.x = 0.0;
msg.position.y = 0.0;
msg.position.z = 0.0;

% Envia para o1
% Gazebo (via ROS 2 bridge)
send(pub, msg);

% CRIANDO O PUBLICADOR EM "/cmd_vel"
% Nó local 
cmdNode = ros2node("/cmdNode");

% Se alistando como publicador em '/cmd_vel'
cmdPub = ros2publisher(cmdNode,"/cmd_vel", "geometry_msgs/Twist");

% Configurando mensagem a ser enviada
cmdVels = ros2message(cmdPub);
cmdVels.linear.x = 0.0;
cmdVels.linear.y = 0.0;
cmdVels.linear.z = 0.0;
cmdVels.angular.x = 0.0;
cmdVels.angular.y = 0.0;
cmdVels.angular.z = 0.0;

% CRIANDO O INSCRITO EM "/scan"
scanNode = ros2node("/scanNode");
pause(3);
% scanSub = ros2subscriber(scanNode, "/scan", @assignDist, "Reliability", "besteffort");
scanSub = ros2subscriber(scanNode, "/scan", "Reliability", "besteffort");

% global right_dist
% global front_right_dist
% 
% global indexOfRight
% global indexOfFrontRight

indexOfRight = round((2.2689 - pi/2) / (0.0071));
indexOfFrontRight = round((2.2689 - pi/4) / (0.0071));

rate = ros2rate(cmdNode, 10);   % 10 Hz

fprintf('Experimento: conectado ao CoppeliaSim e handles obtidos.\n');

%% ================= PARÂMETROS DO EXPERIMENTO =======================
deltaT          = 0.1;        % tempo de amostragem [s]
simTime         = 120;         % duração da simulação [s] (2 minutos)
nSteps          = round(simTime * 6);

ref_dist        = 1.7;          % setpoint da distância lateral [cm]

vel_base         = 10.0;        % velocidade fixa da roda direita
vel_min_roda_dir = -20.0;
vel_max_roda_dir = 20.0;
fact_vel         = 1.0;        % fator de escala da velocidade

maxRange_m       = 10;        % alcance máximo do sensor [m]
dist_segura_cm   = 0.5;         % limiar de segurança para o sensor 45° [cm]

% Pré-alocação de vetores de log
right_dist       = zeros(1, nSteps);     % medição (cm) do sensor 45°
front_right_dist = zeros(1, nSteps);     % medição (cm) do sensor 90°
u                = zeros(1, nSteps);     % ação de controle (vel. roda esquerda)
error            = zeros(1, nSteps);     % erro de distância (com base no 90°)

% Métricas adicionais
satCount  = 0;                 % contador de saturação do controle
nearCount = 0;                 % leituras muito próximas da parede
rough     = 0;                 % rugosidade do controle

%% ============= INICIALIZAÇÃO DO PID E FILTRO =====================
tau_f     = 0.5;                        % constante de tempo do filtro [s]
unomenosA = exp(-(deltaT/tau_f));
alfaana   = 1 - unomenosA;

interror  = 0;   % integral do erro
f_prev    = 0;   % termo filtrado anterior
u_prev    = 0;   % controle anterior (para rugosidade)

%% =================== LOOP DE CONTROLE =============================
tictocStart = tic;
% global i;
% 
for i = 1:nSteps
    
    scan = scanSub.LatestMessage;

    if isempty(scan)
        pause(deltaT);
        continue;
    end
    
    right_dist(i) = scan.ranges(indexOfRight);
    front_right_dist(i) = scan.ranges(indexOfFrontRight);

    if right_dist(i) > 10
        right_dist(i) = 10;
    end

    error(i) = ref_dist - right_dist(i);

    vgb = error(i)

    % Contagem de aproximações perigosas
    if right_dist(i) < 20
        nearCount = nearCount + 1;
    end

    % ---- Integral do erro ----
    interror = interror + error(i);

    % ---- Filtro exponencial no erro ----
    if i == 1
        f_cur   = error(i);
        d_error = 0;
    else
        f_cur   = unomenosA * f_prev + alfaana * error(i);
        d_error = (f_cur - f_prev) / deltaT;
    end
    f_prev = f_cur;

    % ---- PID (roda esquerda) ----
    u(i) = Kp*error(i) + Ki*interror*deltaT + Kd*d_error;

    % ---- Saturação e contagem ----
    if u(i) > vel_max_roda_dir
        u(i) = vel_max_roda_dir;
        satCount = satCount + 1;
    elseif u(i) < vel_min_roda_dir
        u(i) = vel_min_roda_dir;
        satCount = satCount + 1;
    end

    % ---- Rugosidade do controle ----
    if i > 1
        rough = rough + (u(i) - u_prev)^2;
    end
    u_prev = u(i);

    % ---- Lógica de emergência usando sensor 45° ----
    if (front_right_dist(i) < right_dist(i))% && (d45_cm(i) < dist_segura_cm)
        % Obstáculo mais perto na diagonal esquerda -> girar para direita
        v_left  =  2.0;
        v_right = 0.01;
    else
        % Controle normal de seguimento de parede
        v_left  = fact_vel * vel_base;
        v_right = fact_vel * vel_base + u(i);
    end
    
    cmdVels.linear.x = (v_left + v_right) / 2;
    cmdVels.linear.y = 0.0;
    cmdVels.linear.z = 0.0;
    cmdVels.angular.x = 0.0;
    cmdVels.angular.y = 0.0;
    cmdVels.angular.z = (v_right - v_left) / 2;
    
    send(cmdPub, cmdVels);
    % waitfor(rate);

    pause(deltaT);  % mantém a taxa de amostragem
end
t_elapsed = toc(tictocStart);
tempo_formatado = datestr(seconds(t_elapsed), 'HH:MM:SS');
disp('Tempo de simulação (parede):');
disp(tempo_formatado);

%% =================== FINALIZAÇÃO ==================================
% Para o robô ao final do experimento

%% =================== CÁLCULO DAS MÉTRICAS ==========================
t_vec = (0:nSteps-1) * deltaT;           % tempo em segundos
setpoint = ref_dist * ones(1, nSteps);   % referência

IAE = sum(abs(error)) * deltaT;
SSE = error(end);

fprintf('\nMétricas numéricas:\n');
fprintf('  IAE        = %.4f\n', IAE);
fprintf('  SSE        = %.4f\n', SSE);
fprintf('  satCount   = %d\n',   satCount);
fprintf('  nearCount  = %d\n',   nearCount);
fprintf('  rough      = %.4f\n', rough);

%% =================== PLOTS =========================================
% Distâncias e setpoint
figure;
plot(t_vec, d45_cm, 'b', 'LineWidth', 1.2); hold on;
plot(t_vec, d90_cm, 'g', 'LineWidth', 1.2);
plot(t_vec, setpoint,  'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Distância ao obstáculo (cm)');
title(sprintf('Seguimento de parede - Alg: %s | Cenário: %s', algTag, cenarioTag));
legend('Distância 45^\circ', 'Distância 90^\circ', 'Setpoint','Location','best');

% Ação de controle e erro
figure;
yyaxis left;
plot(t_vec, u, 'b', 'LineWidth', 1.2);
ylabel('Velocidade roda esquerda');

yyaxis right;
plot(t_vec, error, 'r', 'LineWidth', 1.0);
ylabel('Erro de distância (cm)');

grid on;
xlabel('Tempo (s)');
title(sprintf('Controle PID - Alg: %s | Cenário: %s', algTag, cenarioTag));
legend('Velo roda esquerda', 'Erro de distância','Location','best');

% %% =================== SALVAMENTO AUTOMÁTICO DAS FIGURAS ==========================
% figDistNamePNG  = sprintf('fig_dist_%s_%s.png',  algTag, cenarioTag);
% figCtrlNamePNG  = sprintf('fig_ctrl_%s_%s.png',  algTag, cenarioTag);
% 
% % Salva a figura da distância (primeira figura)
% figure(1);
% set(gcf,'PaperPositionMode','auto');
% saveas(gcf, figDistNamePNG);
% 
% % Salva a figura da ação de controle (segunda figura)
% figure(2);
% set(gcf,'PaperPositionMode','auto');
% saveas(gcf, figCtrlNamePNG);
% 
% fprintf('\nFiguras salvas como:\n  %s\n  %s\n', ...
%         figDistNamePNG, figCtrlNamePNG);
% 
% 
% %% =================== SALVAMENTO DOS DADOS ==========================
% logFileName = sprintf('log_%s_%s.mat', algTag, cenarioTag);
% save(logFileName, ...
%      'd45_cm','d90_cm','u','error','deltaT','Kp','Ki','Kd', ...
%      'IAE','SSE','rough','nearCount','satCount','t_vec','ref_dist');
% 
% fprintf('\nLog salvo em: %s\n', logFileName);

% 
% function assignDist(scanDataNew)
%     global right_dist;
%     global front_right_dist;
%     global indexOfRight;
%     global indexOfFrontRight;
%     global i;
% 
%     right_dist(i) = scanDataNew.ranges(indexOfRight);
%     front_right_dist(i) = scanDataNew.ranges(indexOfFrontRight);
%     right_dist(i)
% end 