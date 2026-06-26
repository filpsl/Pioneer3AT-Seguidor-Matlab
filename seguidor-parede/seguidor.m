% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %
% % Experimento de seguimento de parede com o robo Pioneer3AT no CoppeliaSim
% % usando controlador PID e um LiDAR.
% %
% % - Sensores:
% %     * LiDAR: [270 graus] Distancia ate a parede na esquerda.
% %     * LiDAR: [315 graus] Distancia ate a parede na frente-esquerda.
% %
% % - Atuadores:
% %     * Roda direita: Velocidade base
% %     * Roda esquerda: Velocidade base - saida do controlador PID.
% %
% % - Este script:
% %     * Pergunta qual algoritmo foi usado na sintonia (PSO / FLA / manual)
% %       e seleciona o conjunto de ganhos correspondente.
% %       usado no nome do arquivo de log (log_PSO_L.mat, etc.).
% %     * Roda o algoritmo por 2 minutos (tempo de robo) com Ts = 0.05 s.
% %     * Plota distancias, erro e velocidade da roda esquerda.
% %     * Salva os dados em um arquivo .mat para analise posterior.
% %
% % Codigo base: Mario Andrés Pastrana Triana (Out-25)
% % Modificado e expandido por: Sérgio Cruz (Dez-25)
% % Modificado e expandido por: Sérgio Cruz e Filipe Barbosa (May-26)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;
addpath("/usr/local/Aria/matlab");
clc;
arrobot_disconnect;

%% =================== SELE��O DO ALGORITMO E CEN�RIO =======================
disp('Selecione o conjunto de ganhos PID:');
disp('  1 - PSO');
disp('  2 - FLA');
disp('  3 - EMP');
disp('  4 - Manual (definir na mao)');
algChoice = input('Opcaoo (1/2/3): ');

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
    case 4
        algTag = 'EMP2';
        % Ganhos do controlador PID - FLA
        Kp = 0.93236*10;
        Ki = 0.3;
        Kd = 0.12713*10;
    otherwise
        algTag = 'MANUAL';
        fprintf('Informe os ganhos manuais do PID:\n');
        Kp = input('  Kp = ');
        Ki = input('  Ki = ');
        Kd = input('  Kd = ');
end

fprintf('\nAlgoritmo: %s', algTag);
fprintf('Gains PID: Kp=%.5f, Ki=%.5f, Kd=%.5f\n\n', Kp, Ki, Kd);
cenarioTag = 'Parede';

%% ============== INICIALIZACAOO PORTA UDP (LIDAR) =======================

porta_lidar = 5000;
udp_lidar = udpport("datagram", "IPV4", "LocalPort", porta_lidar);
flush(udp_lidar);
disp('Escutando LIDAR... Pressione Ctrl+C para parar.');

%% ============== INICIALIZACAO PIONEER =======================
aria_init('-rh', '192.168.0.18');
arrobot_connect

%% ================= PARAMETROS DO EXPERIMENTO =======================
deltaT          = 0.05;        % tempo de amostragem [s]
simTime         = 200;         % duracao da simulacao [s] (2 minutos)
nSteps          = round(simTime * 6);

ref_dist        = 55;          % setpoint da distancia lateral [cm]

linear_velocity = 100.0;
omega_max        = 200.0;
fact_vel         = 2;        % fator de escala da velocidade

% maxRange_m       = 0.9;        % alcance maximo do sensor [m]
dist_segura_cm   = 40;         % limiar de seguranca para o sensor 45 [cm]

% Pre-alocacao de vetores de log
det45_cm = zeros(1, nSteps);     % medicao (cm) do sensor 45
det90_cm = zeros(1, nSteps);     % medicao (cm) do sensor 90
det45_mm = 0.0;
det90_mm = 0.0;

u      = zeros(1, nSteps);     % acaoo de controle (vel. roda esquerda)
error  = zeros(1, nSteps);     % erro de distancia (com base no 90)
const_pid = 4;
error_max = 60;

% Metricas adicionais
satCount  = 0;                 % contador de saturacao do controle
nearCount = 0;                 % leituras muito proximas da parede
rough     = 0;                 % rugosidade do controle

%% ============= INICIALIZACAOO DO PID E FILTRO =====================
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
    dists = get_lidar(udp_lidar, [270, 315], 1);
    det90_mm = dists(1);
    det45_mm = dists(2);
    
    % ----- Transformando de mm para cm ---------------
    det90_cm(i) = det90_mm / 10;
    det45_cm(i) = det45_mm / 10;

    % ------ Verificando se eh a primeira iteracao -----
    if det90_mm == -1
        if i == 1
            det90_cm(i) = ref_dist;
        else
            det90_cm(i) = det90_cm(i-1);
        end
    end
    if det45_mm == -1
        if i == 1
            det45_cm(i) = ref_dist;
        else
            det45_cm(i) = det45_cm(i-1);
        end
    end

    % ---- Erro de distancia (sensor 90) ------
    
    error(i) = ref_dist - det90_cm(i);
    
    % 
    % if error(i) > error_max
    %     error(i) = error_max;
    % end
    % 
    % if error(i) < -error_max
    %     error(i) = -error_max;
    % end

    % --- Contagem de aproximacoes perigosas ------
    if det90_cm(i) < 20
        nearCount = nearCount + 1;
    end

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

    % --- Saturacao do Controlador ---
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
    
    % ---- Logica de emergencia usando sensor 45 ----
    if (det45_cm(i) < dist_segura_cm)
        % Obstaculo mais perto na diagonal esquerda -> girar para direita
        v_left  =  fact_vel * 100.0;
        v_right = -fact_vel * 100.0;
    else
        % Controle normal de seguimento de parede
        v_left  = fact_vel * linear_velocity + u(i);
        v_right = fact_vel * linear_velocity;
    end

    arrobot_setwheelvels(v_left, v_right);
    
    fprintf('d45 = %.4f | d90 = %.4f |vL = %.4f | vR = %.4f | U: %.4f \n', det45_cm(i),det90_cm(i),v_left,v_right, u(1,i));

    if int_count == 100
        int_count = 1;
    end

    pause(deltaT);  % mantem a taxa de amostragem
end

arrobot_setwheelvels(0, 0);
arrobot_disable_motors;
arrobot_disconnect;

% 
% %% =================== CaLCULO DAS MeTRICAS ==========================
t_vec = (0:nSteps-1) * deltaT;           % tempo em segundos
setpoint = ref_dist * ones(1, nSteps);   % referencia

IAE = sum(abs(error)) * deltaT;
SSE = error(end);

fprintf('\nMatricas numericas:\n');
fprintf('  IAE        = %.4f\n', IAE);
fprintf('  SSE        = %.4f\n', SSE);
fprintf('  satCount   = %d\n',   satCount);
fprintf('  nearCount  = %d\n',   nearCount);
fprintf('  rough      = %.4f\n', rough);

%% =================== PLOTS =========================================
% Distancias e setpoint
figure;
plot(t_vec, det45_cm, 'b', 'LineWidth', 1.2); hold on;
plot(t_vec, det90_cm, 'g', 'LineWidth', 1.2);
plot(t_vec, setpoint,  'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Distancia ao obstaculo (cm)');
title(sprintf('Seguimento de parede - Alg: %s', algTag));
legend('Distancia 45^\circ', 'Distancia 90^\circ', 'Setpoint','Location','best');

% Acaoo de controle e erro
figure;
yyaxis left;
plot(t_vec, u, 'b', 'LineWidth', 1.2);
ylabel('Velocidade roda esquerda');

yyaxis right;
plot(t_vec, error, 'r', 'LineWidth', 1.0);
ylabel('Erro de distancia (cm)');

grid on;
xlabel('Tempo (s)');
title(sprintf('Controle PID - Alg: %s | Cenario: %s', algTag, cenarioTag));
legend('Velo roda esquerda', 'Erro de distancia','Location','best');

%% =================== SALVAMENTO AUTOMATICO DAS FIGURAS ==========================
timestamp       = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
figDistNamePNG  = sprintf('fig_dist_%s_%s_%s.png', algTag, cenarioTag, timestamp);
figCtrlNamePNG  = sprintf('fig_ctrl_%s_%s_%s.png', algTag, cenarioTag, timestamp);

figPath = "resultados";

% Salva a figura da distancia (primeira figura)
figure(1);
set(gcf,'PaperPositionMode','auto');
saveas(gcf, fullfile(figPath, figDistNamePNG));

% Salva a figura da acao de controle (segunda figura)
figure(2);
set(gcf,'PaperPositionMode','auto');
saveas(gcf, fullfile(figPath, figCtrlNamePNG));

fprintf('\nFiguras salvas como:\n  %s\n  %s\n', ...
        figDistNamePNG, figCtrlNamePNG);


%% =================== SALVAMENTO DOS DADOS ==========================
logFileName = sprintf('log_%s_%s_%s.mat', algTag, cenarioTag, timestamp);
logFilePath = "resultados";

save(fullfile(logFilePath, logFileName), ...
     'det45_cm','det90_cm','u','error','deltaT','Kp','Ki','Kd', ...
     'IAE','SSE','rough','nearCount','satCount','t_vec','ref_dist');

fprintf('\nLog salvo em: %s\n', logFileName);
