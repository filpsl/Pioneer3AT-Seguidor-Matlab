% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %
% % Experimento de locomover no meio do corredor de forma autônoma com o robô 
% % Pioneer3AT no CoppeliaSim usando controlador PID e um LiDAR.
% %
% % - Sensores:
% %     * LiDAR: Escaneamento 360 graus, 0.05m até 12 metros.
% %         
% % - Atuadores:
% %     * Roda direita: velocidade fixa
% %     * Roda esquerda: velocidade modulada pelo PID
% %
% % - Este script:
% %     * Se conecta com o Pioneer3AT no CoppeliaSim, e se move de forma
% %       a ficar sempre no meio do corredor ou ambiente.
% %
% % Código base: Mario Andrés Pastrana Triana (Out-25)
% % Modificado e expandido por: Sérgio Cruz e Filipe Barbosa (Maio-06-2026)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear;
clc;

%% ============== INICIALIZAÇÃO COPPELIASIM =======================
vrep = remApi('remoteApi');
vrep.simxFinish(-1);  % fecha conexões antigas

clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if clientID < 0
    warning('Experimento: não foi possível conectar ao CoppeliaSim.');
    return;
end

% Rodas
[~, h_fl] = vrep.simxGetObjectHandle( ...
    clientID,'front_left_wheel',  vrep.simx_opmode_blocking);
[~, h_fr] = vrep.simxGetObjectHandle( ...
    clientID,'front_right_wheel', vrep.simx_opmode_blocking);
[~, h_rl] = vrep.simxGetObjectHandle( ...
    clientID,'rear_left_wheel',   vrep.simx_opmode_blocking);
[~, h_rr] = vrep.simxGetObjectHandle( ...
    clientID,'rear_right_wheel',  vrep.simx_opmode_blocking);

% LiDAR
[rc1, lidar] = vrep.simxGetObjectHandle( ...
    clientID, 'Hokuyo_URG_04LX_UG01_laser', vrep.simx_opmode_blocking);
if rc1 ~= vrep.simx_return_ok
    warning('Experimento: problema ao obter handler do LiDAR.');
end

fprintf('Experimento: conectado ao CoppeliaSim e handles obtidos.\n');
%% ==================== VARIÁVEIS GLOBAIS ============================%%
% Tempo
dt = 0.1;

% Controlador
kp = 1;
kd = 0.02;
ki = 0.01;
filter_alpha = 0.5;
filtered_err = 0;
previous_err = 0;
integral_err = zeros(1, 30);
integral_count = 1;

% Velocidade
linear_speed = 2;

%% ============== LOOP PRINCIPAL ============================================

while true
    %% -- Leitura do LiDAR --------------------------------------------------
    % Dados publicados pelo Lua como XYZ empacotados (3 floats por ponto).
    [rc2, raw] = vrep.simxGetStringSignal( ...
        clientID, 'lidarData', vrep.simx_opmode_blocking);

    if rc2 ~= vrep.simx_return_ok || isempty(raw)
        fprintf('Aguardando sinal lidarData... (rc = %d)\n', rc2);
        pause(0.1);
        continue;
    end

    dist_pts = vrep.simxUnpackFloats(raw);      % 1 float por raio (rangeData=true)
    n_pts    = numel(dist_pts);

    % Reconstrói ângulos por índice (-120° a +120°, espaçamento uniforme)
    theta = linspace(0, 240, n_pts);         % graus

    % Janela angular usada para estimar cada lado (ajuste se necessário)
    janela_deg = 5;

    % Lado direito: pontos ao redor de -90°
    mask_dir = theta > (45 - janela_deg) & theta < (45 + janela_deg);
    dist_dir = mean(dist_pts(mask_dir));        % distância média lado direito (m)

    % Lado esquerdo: pontos ao redor de +90°
    mask_esq = theta > ( 225 - janela_deg) & theta < ( 225 + janela_deg);
    dist_esq = mean(dist_pts(mask_esq));        % distância média lado esquerdo (m)

    fprintf('dist_dir = %.3f m  |  dist_esq = %.3f m\n', dist_dir, dist_esq);
    
    % Cálculo do erro atual
    err = dist_esq - dist_dir;
  
    if integral_count == 30
      integral_count = 1
    end

    % Controlador PID
    proportional_term = kp * err;
    
    filtered_err = (1 - filter_alpha) * err + (filter_alpha * previous_err);
    derivative_term = filtered_err / dt;
    previous_err = err;

    integral_err(integral_count) = err;
    integral_count = integral_count + 1;
    integral_term = sum(integral_err);

    omega = proportional_term + derivative_term + integral_term;

    % Calcular as velocidades de cada Roda
    right_speed = linear_speed + omega;
    left_speed = linear_speed - omega;

    % Enviar as velocidades
    vrep.simxSetJointTargetVelocity(clientID, h_fl, left_speed,  vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, h_rl, left_speed,  vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, h_fr, right_speed, vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, h_rr, right_speed, vrep.simx_opmode_oneshot);
    
    pause(dt);
end

vrep.simxFinish(clientID);
vrep.delete();
