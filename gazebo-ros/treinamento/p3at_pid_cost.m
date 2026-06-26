% p3at_pid_cost  -  Função de custo para ajuste de PID do Pioneer3AT no CoppeliaSim
%
% Uso:
%   J = p3at_pid_cost([Kp Ki Kd]);
%
% Descrição:
%   - Usa DOIS sensores ultrassônicos:
%       * h_us45: sensor a ~45° apontado para a parede (frente-esquerda)
%       * h_us90: sensor a ~90° (lateral), medindo diretamente a distância à parede
%   - O erro do PID é calculado com base no sensor a 90° (distância lateral).
%   - O custo J considera:
%       * IAE (Integral do erro absoluto de distância lateral)
%       * Saturações da ação de controle
%       * Aproximações perigosas da parede (qualquer sensor < dist_min_cm)
%       * Rugosidade do controle (variação de u)
%
% Requisitos:
%   - CoppeliaSim aberto com a cena do Pioneer3AT.
%   - Nomes corretos dos sensores de 45° e 90° (ajuste abaixo).
%
% Autor: Sérgio Cruz
% Data: Dez-25

function J = p3at_pid_cost(K)
   
    %% ============ AJEITANDO O GAZEBO =================
    % Dados do Mundo
    worldName = 'mezanino_graco';
    modelName = 'pioneer3at';

    % Inicia o nó do MATLAB para se comunicar com os nós do ROS2
    node = ros2node("/matlab_pose_controller");
    
    % Cria o publisher mirando no tópico mapeado pela ponte
    pub = ros2publisher(node, "model/pioneer3at/pose", "geometry_msgs/Pose");
    msg = ros2message(pub);

    % Configura a posição
    msg.position.x = 0.0;
    msg.position.y = 0.0;
    msg.position.z = 0.0;

    % Envia para o Gazebo via ROS2 Bridge
    send(pub, msg);

    % Criando o nó publicador
    cmdNode = ros2node('cmdNode');

    % Se alistando como publicador em 'cmd_vel'
    cmdPub = ros2publisher(cmdNode, 'cmd_vel', 'geometry_msgs/Twist');

    % Configurando a mensagem a ser enviada
    cmdVels = ros2message(cmdPub);
    cmdVels.linear.x = 0.0;
    cmdVels.linear.y = 0.0;
    cmdVels.linear.z = 0.0;
    cmdVels.angular.x = 0.0;
    cmdVels.angular.y = 0.0;
    cmdVels.angular.z = 0.0;

    % Criando o inscrito em '/scan'
    scanNode = ros2node("scanNode");
    pause(3);
    scanSub = ros2subscriber(scanNode, "/scan", "Reliability", "besteffort");
    indexOfRight = abs(round((2.2689 + pi/2) / (0.0071)));
    indexOfFrontRight = round((2.2689 + pi/4) / (0.0071));

    rate = ros2rate(cmdNode, 10);   % 10 Hz

    fprintf('Experimento: conectado ao Gazebo e comunicações estabelecidas.\n');

    %% ================= PARÂMETROS DO EXPERIMENTO =======================
    deltaT          = 0.1;   % tempo de amostragem [s]
    segundos        = 24;       % Duração da simulação
    tic_simul       = round(segundos * 9.883);
    ref_dist        = 0.55;     % setpoint da distância lateral [m]
    
    base_vel         = 0.10;    % velocidade fixa da roda direita
    omega_max        = 0.20;
    fact_vel         = 2;     % Fator de multiplicação da velocidade (andar mais/menos rápido)
    
    safe_dist      = 0.4;    % distância mínima aceitável da parede [m]
    
    maxRange_m      = 15;    % alcance máximo considerado do sensor [m]
    %theta45         = deg2rad(45);  % ângulo do sensor de 45°
    
    
    %% ============= INICIALIZAÇÃO DO PID E MÉTRICAS =====================
    % Garante que K é linha
    K  = K(:).';
    % Ganhos do controlador PID
    Kp = K(1)  % Ganho Proporcional
    Ki = K(2)  % Ganho Integral
    Kd = K(3)  % Ganho Derivativo

    % Filtro exponencial simples no erro (fixo, robusto)
    tau_f     = 0.5;                        % constante de tempo [s]
    unomenosA = exp(-(deltaT/tau_f));
    alfaana   = 1 - unomenosA;
    
    interror  = zeros(1, 50);   % integral do erro
    int_count = 1;
    f_prev    = 0;   % termo filtrado anterior
    u_prev    = 0;   % ação de controle anterior

    % Função Custo (J)
    % J = peso_IAE * IAE + ...
    %     peso_sat * satCount + ...
    %     peso_proximidade * nearCount + ...
    %     peso_rugos * rough;
    peso_IAE         = 0.6;     % peso do erro acumulado
    IAE              = 0;       % Integral do erro absoluto da distância lateral
    peso_sat         = 0.1;     % peso de saturação
    satCount         = 0;       % contagem de saturações
    peso_proximidade = 0.2;    % peso para ficar muito perto da parede
    nearCount        = 0;       % contagem de leituras muito próximas da parede
    peso_rugos       = 0.1;    % peso de rugosidade do controle    
    rough            = 0;       % rugosidade do controle
    
    %% =================== LOOP DE CONTROLE ===============================
    %tic;
    for i = 1:tic_simul
    
        % -----------------------------------------------------------------
        % Leitura do lidar a 45°
        % -----------------------------------------------------------------
        scan = scanSub.LatestMessage;

        if isempty(scan)
          pause(deltaT);
          continue;
        end

        right_dist = scan.ranges(indexOfRight);
        front_right_dist = scan.ranges(indexOfFrontRight);

        if right_dist > 10
          right_dist = 10;
        end
        
        % Cálculo do ERRO
        error = right_dist - ref_dist;
       
        % Cálculo da parte integral
        interror(int_count) = error;
        int_count = int_count + 1;
        
        if int_count > 50
          int_count = 1;
        end
        
        % Cálculo da parte derivativa
        if i == 1 
          f_cur = error;
          d_error = 0;
        else
          f_cur = unomenosA * f_prev + alfaana * error;
          d_error = (f_cur - f_prev) / deltaT;
        end
        f_prev = f_cur;
        
        % PID (roda esquerda)
        u = Kp*error + Ki*sum(interror)*deltaT + Kd*d_error;
        
        % Saturação do controlador 
        if u > omega_max
            u = omega_max;
            satCount = satCount + 1; % Penaliza saturação
        elseif u < -omega_max
            u = -omega_max;
            satCount = satCount + 1; % Penaliza saturação
        end

        % Atualização das velocidades das rodas
        if front_right_dist < safe_dist
          v_left  = -1*(fact_vel * base_vel);     % Rotaciona à direita para evita obstáculo à diagonal-esquerda
          v_right = fact_vel * base_vel;        %

        else            
            v_left  =  fact_vel * base_vel + u;
            v_right =  fact_vel * base_vel - u;
        end

        % Envio das velocidades às rodas
        vel_x = double((v_left + v_right) / 2);
        vel_z = double((v_left - v_right) / 2);

        cmdVels.linear.x = vel_x;
        cmdVels.angular.z = vel_z;

        send(cmdPub, cmdVels);

        % -----------------------------------------------------------------
        % Atualiza métricas de custo
        % -----------------------------------------------------------------
    
        % 1) Erro acumulado (IAE) da distância lateral
        IAE = IAE + abs(error)*deltaT;
    
        % 2) Aproximação perigosa da parede
        %    Penaliza se QUALQUER dos sensores estiver abaixo do limite
        if (right_dist < safe_dist) || (front_right_dist < safe_dist)
            nearCount = nearCount + 1;
        end
    
        % 3) Rugosidade do controle (evita "nervosismo")
        if i > 1
            rough = rough + (u - u_prev)^2;
        end
        u_prev = u;
        
        pause(deltaT);
    end
    %temp_simul = toc
        
    %% =================== FINALIZAÇÃO ==================================
    % Parar o robô ao final do experimento e teleportar para (0, 0, 0)
    cmdVels.linear.x = 0;
    cmdVels.angular.z = 0;

    send(cmdPub, cmdVels);
    pause(0.5);

    teleportModel(worldName, modelName, 9, 3.6, 0.15, pi/2);
    pause(0.5);

    send(cmdPub, cmdVels);
    
    %% =================== CÁLCULO DO CUSTO J ============================
    J = peso_IAE * IAE + ...
        peso_sat * satCount + ...
        peso_proximidade * nearCount + ...
        peso_rugos * rough
end


% % p3at_pid_cost  -  Função de custo para ajuste de PID do Pioneer3AT no CoppeliaSim
% %
% % Uso:
% %   J = p3at_pid_cost([Kp Ki Kd]);
% %
% % Descrição:
% %   - Conecta (na primeira chamada) ao CoppeliaSim via Remote API.
% %   - Reinicia a simulação (stop + start).
% %   - Roda o controlador PID de seguimento de parede por um tempo fixo.
% %   - Calcula um custo J baseado:
% %       * no erro de distância (IAE),
% %       * em quantas vezes a saída saturou,
% %       * em quantas vezes chegou muito perto da parede.
% %
% %   Quanto menor J, melhor o conjunto [Kp Ki Kd].
% %
% % Requisitos:
% %   - CoppeliaSim aberto com a cena do Pioneer3AT.
% %   - Sensor "Pioneer_p3dx_ultrasonicSensor2" configurado a ~45°.
% %
% % Autor: (seu nome)
% % Data: (data)
% 
% function J = p3at_pid_cost(K)
% 
%     % Garante que K é linha
%     K = K(:).';
%     Kp = K(1);
%     Ki = K(2);
%     Kd = K(3);
% 
%     %% ================= PARÂMETROS DO EXPERIMENTO =======================
%     deltaT          = 0.05;   % tempo de amostragem [s]
%     numero_muestras = 50;     % duração da simulação
%     ideal           = 70;     % setpoint da distância [cm]
% 
%     pdi             = 1.0;    % velocidade fixa da roda direita
%     pii_min         = 0.0;
%     pii_max         = 2.0;
% 
%     dist_min_cm      = 20;    % distância mínima aceitável da parede [cm]
%     peso_IAE         = 1.0;   % peso do erro acumulado
%     peso_sat         = 0.1;   % peso de saturação
%     peso_proximidade = 1000;  % peso para ficar muito perto da parede
%     peso_rugos = 0.01;     % peso de rugosidade do controle
%     % peso_IAE         = 0.5;   % peso do erro acumulado
%     % peso_sat         = 0.3;   % peso de saturação
%     % peso_proximidade = 0.15;  % peso para ficar muito perto da parede
%     % peso_rugos = 0.05;     % peso de rugosidade do controle
% 
%     %% ============ CONEXÃO E HANDLES (APENAS NA 1ª VEZ) =================
%     persistent vrep clientID h_fl h_fr h_rl h_rr h_us initialized
% 
%     if isempty(initialized)
%         vrep = remApi('remoteApi');
%         vrep.simxFinish(-1);  % fecha conexões antigas
% 
%         clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
%         if clientID < 0
%             warning('p3at_pid_cost: não foi possível conectar ao CoppeliaSim.');
%             J = 1e9;  % custo enorme pra penalizar
%             return;
%         end
% 
%         % Rodas
%         [~, h_fl] = vrep.simxGetObjectHandle(clientID,'front_left_wheel',  vrep.simx_opmode_blocking);
%         [~, h_fr] = vrep.simxGetObjectHandle(clientID,'front_right_wheel', vrep.simx_opmode_blocking);
%         [~, h_rl] = vrep.simxGetObjectHandle(clientID,'rear_left_wheel',   vrep.simx_opmode_blocking);
%         [~, h_rr] = vrep.simxGetObjectHandle(clientID,'rear_right_wheel',  vrep.simx_opmode_blocking);
% 
%         % Sensor ultrassônico a 45°
%         [rc, h_us] = vrep.simxGetObjectHandle( ...
%             clientID,'Pioneer_p3dx_ultrasonicSensor2', vrep.simx_opmode_blocking);
% 
%         if rc ~= vrep.simx_return_ok
%             warning('p3at_pid_cost: problema ao obter handle do sensor ultrassônico.');
%         end
% 
%         initialized = true;
%         fprintf('p3at_pid_cost: conectado ao CoppeliaSim e handles obtidos.\n');
%    end
% 
%     %% =================== REINICIA A SIMULAÇÃO ===========================
%     % stop + start para garantir condições iniciais iguais
%     %vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
%     %vrep.simxLoadScene(clientID,'C:\Users\sergi\MATLAB Drive\P3AT_CoppeliA_PID\Simulation_MATLAB_Sergio\Pioneer_4_rodas_training.ttt', 0, vrep.simx_opmode_oneshot_wait)
%     %pause(2.0);  % tempo para parar
% 
%     %vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
%     %pause(1.0);  % tempo para estabilizar início
% 
%     %% ============= INICIALIZAÇÃO DO PID E MÉTRICAS =====================
%     % ==== Filtro exponencial simples para o erro (fixo, robusto) ====
%     % Constante de tempo do filtro (em segundos)
%     tau_f = 0.5;                 % ajuste grosso; não depende de Kp, Ki, Kd
%     unomenosA = exp(-(deltaT/tau_f));
%     alfaana   = 1 - unomenosA;
% 
%     interror  = 0;   % integral
%     f_prev    = 0;   % saída anterior do filtro
%     IAE       = 0;   % Integral do erro absoluto
%     satCount  = 0;   % contagem de saturações
%     nearCount = 0;   % contagem de leituras muito próximas da parede
%     rough     = 0;      % rugosidade do controle
%     u_prev    = 0;      % ação de controle anterior
% 
%     %% =================== LOOP DE CONTROLE ===============================
%     tic;
%     for i = 1:numero_muestras
% 
%         % ----- leitura do sensor -----
%         [rtd1, ~, sdd1] = vrep.simxReadProximitySensor( ...
%             clientID, h_us, vrep.simx_opmode_oneshot_wait);
% 
%         % componente Z (como no código original)
%         rsensord1a = sdd1(:,3); % [m]
% 
%         if rsensord1a < 0.0001 || rsensord1a >= 0.8
%             rsensord1a = 0.8;
%         end
% 
%         inputpid = double(rsensord1a * 100); % [cm]
% 
%         % ----- erro de distância -----
%         error = ideal - inputpid;
% 
%         % acumula integral do erro
%         interror = interror + error;
% 
%         % filtro exponencial no erro
%         if i == 1
%             f_cur   = error;
%             d_error = 0;%f_cur / deltaT;
%         else
%             f_cur   = unomenosA * f_prev + alfaana * error;
%             d_error = (f_cur - f_prev) / deltaT;
%         end
%         f_prev = f_cur;
% 
%         % ----- PID (roda esquerda) -----
%         u = Kp*error + Ki*interror*deltaT + Kd*d_error;
% 
%         % saturação
%         if u > pii_max
%             u = pii_max;
%             satCount = satCount + 1;
%         elseif u < pii_min
%             u = pii_min;
%             satCount = satCount + 1;
%         end
% 
%         v_left  = 0.7 * u;
%         v_right = 0.7 * pdi;
% 
%         % envia velocidades
%         vrep.simxSetJointTargetVelocity(clientID, h_fl, v_left,  vrep.simx_opmode_oneshot);
%         vrep.simxSetJointTargetVelocity(clientID, h_rl, v_left,  vrep.simx_opmode_oneshot);
%         vrep.simxSetJointTargetVelocity(clientID, h_fr, v_right, vrep.simx_opmode_oneshot);
%         vrep.simxSetJointTargetVelocity(clientID, h_rr, v_right, vrep.simx_opmode_oneshot);
% 
%         % ----- acumula custo -----
%         IAE = IAE + abs(error)*deltaT;
% 
%         if inputpid < dist_min_cm
%             nearCount = nearCount + 1;
%         end
%         % ----- Rugosidade do controle
%         if i > 1
%             rough = rough + (u - u_prev)^2;
%         end
%         u_prev = u;
% 
%         pause(deltaT);
%     end
%     temp_simul = toc
% 
%     % para o robô ao final desta avaliação
%     vrep.simxSetJointTargetVelocity(clientID,h_fl,0,vrep.simx_opmode_oneshot);
%     %pause(0.5);
%     vrep.simxSetJointTargetVelocity(clientID,h_rl,0,vrep.simx_opmode_oneshot);
%     %pause(0.5);
%     vrep.simxSetJointTargetVelocity(clientID,h_fr,0,vrep.simx_opmode_oneshot);
%     %pause(0.5);
%     vrep.simxSetJointTargetVelocity(clientID,h_rr,0,vrep.simx_opmode_oneshot);
%     %pause(0.5);
% 
%     %% =================== CÁLCULO DO CUSTO J ============================
%     J = peso_IAE * IAE + ...
%         peso_sat * satCount + ...
%         peso_proximidade * nearCount + ...
%         peso_rugos * rough;
% end
