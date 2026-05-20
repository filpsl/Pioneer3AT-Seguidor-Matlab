% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %
% % Experimento de manter o posicionamento do robô Pioneer3AT sempre no meio 
% % de um corredor. (Distância entre a parede esquerda = Distância entre a
% % parede direita.) usando controlador PID e um LiDAR.
% %
% % - Sensores:
% %     * LiDAR: [90 e 270 graus] Direita e esquerda, respectivamente.
% %     * LiDAR: [45 e 315 graus] Frente direita e frente esquerda, respectiva-
% %       mente.
% %
% % - Atuadores:
% %     * Roda direita: Velocidade base - saída do controlador PID
% %     * Roda esquerda: Velocidade base + saída do controlador PID
% %
% % - Este script:
% %     * Pergunta qual algoritmo foi usado na sintonia (PSO / FLA / manual)
% %       e seleciona o conjunto de ganhos correspondente.
% %     * Roda a simulação por 2 minutos (tempo de robô) com Ts = 0.05 s.
% %     * Plota distâncias, erros e velocidades das rodas.
% %     * Salva os dados em um arquivo .mat para análise posterior.
% %
% % Código base: Mario Andrés Pastrana Triana (Out-25)
% % Modificado e expandido por: Sérgio Cruz (Dez-25)
% % Modificado e expandido por: Sérgio Cruz e Filipe Barbosa (May-26)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;
addpath("/usr/local/Aria/matlab");
clc;
arrobot_disconnect;

%% =================== SELEÇÃO DO ALGORITMO E CENÁRIO =======================
disp('Selecione o conjunto de ganhos PID:');
disp('  1 - PSO');
disp('  2 - FLA');
disp('  3 - EMP');
disp('  4 - Manual (definir na mão)');
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
    case 3
        algTag = 'EMP';
        % Ganhos do controlador PID - FLA
        Kp = 0.63236*10;
        Ki = 0.1;
        Kd = 0.09713*10;
    otherwise
        algTag = 'MANUAL';
        fprintf('Informe os ganhos manuais do PID:\n');
        Kp = input('  Kp = ');
        Ki = input('  Ki = ');
        Kd = input('  Kd = ');
end

fprintf('\nAlgoritmo: %s', algTag);
fprintf('Gains PID: Kp=%.5f, Ki=%.5f, Kd=%.5f\n\n', Kp, Ki, Kd);
cenarioTag = 'corredor';

%% ============== INICIALIZAÇÃO PORTA UDP (LIDAR) =======================

porta_lidar = 5000;
udp_lidar = udpport("datagram", "IPV4", "LocalPort", porta_lidar);
flush(udp_lidar);
disp('Escutando LIDAR... Pressione Ctrl+C para parar.');

%% ============== INICIALIZAÇÃO PIONEER =======================
aria_init('-rh', '192.168.0.3');
arrobot_connect

%% ================= PARÂMETROS DO EXPERIMENTO =======================
deltaT          = 0.05;        % tempo de amostragem [s]
simTime         = 300;         % duração da simulação [s] (2 minutos)
nSteps          = round(simTime * 6);

ref_dist        = 60;          % setpoint da distância lateral [cm]

linear_velocity  = 100.0;
omega_max = 100.0;
fact_vel         = 1;        % fator de escala da velocidade

maxRange_m       = 0.9;        % alcance máximo do sensor [m]
dist_segura_cm   = 40;         % limiar de segurança para o lidar° [cm]

% Pré-alocação de vetores de log
left_dist = zeros(1, nSteps);     % medição (cm) do sensor 45°
front_left_dist = zeros(1, nSteps);
right_dist = zeros(1, nSteps);
front_right_dist = zeros(1, nSteps);

left_dist_mm = 0;     % medição (cm) do sensor 45°
front_left_dist_mm = 0;
right_dist_mm = 0;
front_right_dist_mm = 0;

u      = zeros(1, nSteps);     % ação de controle (vel. roda esquerda)
error  = zeros(1, nSteps);     % erro de distância (com base no 90°)
const_pid = 4;
error_max = 60;

% Métricas adicionais
satCount  = 0;                 % contador de saturação do controle
nearCount = 0;                 % leituras muito próximas da parede
rough     = 0;                 % rugosidade do controle

%% ============= INICIALIZAÇÃO DO PID E FILTRO =====================
tau_f     = 0.5;                        % constante de tempo do filtro [s]
unomenosA = exp(-(deltaT/tau_f));
alfaana   = 1 - unomenosA;

interror  = zeros(1,50);   % integral do erro
int_count = 1;
f_prev    = 0;   % termo filtrado anterior
u_prev    = 0;   % controle anterior (para rugosidade)

%% =================== LOOP DE CONTROLE =============================
% tictocStart = tic;
for i = 1:nSteps

    % ---------------------------------------------------------------
    % Leitura do LIDAR
    % ---------------------------------------------------------------
    dists = get_lidar(udp_lidar, [270, 315, 90, 45], 1);

    left_dist_mm = dists(1);
    front_left_dist_mm = dists(2);
    right_dist_mm = dists(3);
    front_right_dist_mm = dists(4);

    % ----- Transformando de mm para cm ---------------
   left_dist(i) = left_dist_mm / 10;
   front_left_dist(i) = front_left_dist_mm / 10;
   right_dist(i) = right_dist_mm / 10;
   front_right_dist(i) = front_right_dist_mm / 10;

    % ------ Verificando se é a primeira iteração -----
    if left_dist_mm == -1
        if i == 1
            left_dist(i) = ref_dist;
        else
            left_dist(i) = left_dist(i-1);
        end
    end
    
    if front_left_dist_mm == -1
        if i == 1
            front_left_dist(i) = ref_dist;
        else
            front_left_dist(i) = front_left_dist(i-1);
        end
    end

    if right_dist_mm == -1
        if i == 1
            right_dist(i) = ref_dist;
        else
            right_dist(i) = right_dist(i-1);
        end
    end

    if front_right_dist_mm == -1
        if i == 1
            front_right_dist(i) = ref_dist;
        else
            front_right_dist(i) = front_right_dist(i-1);
        end
    end

    % ---- Erro de distância (sensor 90°) ------
    error(i) = right_dist(i) - left_dist(i);

    % ---- Integral do erro ----
    interror(int_count) = error(i);
    int_count = int_count + 1;

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
    u(i) = Kp*error(i) + Ki*sum(interror)*deltaT + Kd*d_error;

    % ---- Saturação e contagem ----
    if u(i) > omega_max
        u(i) = omega_max;
        satCount = satCount + 1;
    elseif u(i) < -omega_max
        u(i) = -omega_max;
        satCount = satCount + 1;
    end

    % ---- Rugosidade do controle ----
    if i > 1
        rough = rough + (u(i) - u_prev)^2;
    end
    u_prev = u(i);
    % fprintf("U: %.4f\n", u(i));
    
    % ---- Lógica de emergência usando sensor 45° ----
    if front_right_dist(i) < dist_segura_cm
        % Obstáculo mais perto na diagonal esquerda -> girar para direita
        v_left  = -fact_vel * 100.0;
        v_right = fact_vel * 100.0;

    elseif front_left_dist(i) < dist_segura_cm
        v_left  = fact_vel * 100.0;
        v_right = -fact_vel * 100.0;
    else
        % Controle normal de seguimento de parede
   
        v_left  = fact_vel * linear_velocity + u(i);
        v_right = fact_vel * linear_velocity - u(i) ;
    end

    arrobot_setwheelvels(v_left, v_right);
    
    fprintf('fr = %.4f | r = %.4f |fl = %.4f | l = %.4f |vL = %.4f | vR = %.4f | U: %.4f \n', front_right_dist(i), right_dist(i), front_left_dist(i), left_dist(i),v_left,v_right, u(1,i));
    
   
    if int_count == 100
        int_count = 1;
    end

    pause(deltaT);  % mantém a taxa de amostragem
end

arrobot_setwheelvels(0, 0);
arrobot_disable_motors;
arrobot_disconnect;

% 
% %% =================== CÁLCULO DAS MÉTRICAS ==========================
t_vec = (0:nSteps-1) * deltaT;           % tempo em segundos
setpoint = ref_dist * ones(1, nSteps);   % referência

IAE = sum(abs(error)) * deltaT;
SSE = error(end);

fprintf('\nMétricas numéricas:\n');
fprintf('  IAE        = %.4f\n', IAE);
fprintf('  SSE        = %.4f\n', SSE);
fprintf('  satCount   = %d\n',   satCount);
fprintf('  rough      = %.4f\n', rough);

%% =================== PLOTS =========================================
% Distâncias e setpoint
figure;
plot(t_vec, front_left_dist, 'b', 'LineWidth', 1.2); hold on;
plot(t_vec, left_dist, 'g', 'LineWidth', 1.2);
plot(t_vec, setpoint,  'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Distância ao obstáculo (cm)');
title(sprintf('Seguimento de parede - Alg: %s', algTag));
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

%% =================== SALVAMENTO AUTOMÁTICO DAS FIGURAS ==========================
figDistNamePNG  = sprintf('fig_dist_%s_%s.png',  algTag, cenarioTag);
figCtrlNamePNG  = sprintf('fig_ctrl_%s_%s.png',  algTag, cenarioTag);

% Salva a figura da distância (primeira figura)
figure(1);
set(gcf,'PaperPositionMode','auto');
saveas(gcf, figDistNamePNG);

% Salva a figura da ação de controle (segunda figura)
figure(2);
set(gcf,'PaperPositionMode','auto');
saveas(gcf, figCtrlNamePNG);

fprintf('\nFiguras salvas como:\n  %s\n  %s\n', ...
        figDistNamePNG, figCtrlNamePNG);


%% =================== SALVAMENTO DOS DADOS ==========================
logFileName = sprintf('log_%s_%s.mat', algTag, cenarioTag);
save(logFileName, ...
     'left_dist','front_left_dist','right_dist','front_right_dist','u','error','deltaT','Kp','Ki','Kd', ...
     'IAE','SSE','rough','nearCount','satCount','t_vec','ref_dist');

fprintf('\nLog salvo em: %s\n', logFileName);
