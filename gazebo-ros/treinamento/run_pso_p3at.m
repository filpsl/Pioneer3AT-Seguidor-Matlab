%% run_pso_p3at.m
% Script para ajustar [Kp Ki Kd] do controlador PID do Pioneer3AT
% usando o Particle Swarm Optmization (PSO) e a simulação no CoppeliaSim.

clear; clc; close all;

% ------------------- Parâmetros do PSO ---------------------
NoParticles = 5;
cfg.iter  = 30;
cfg.pso.c1 = 2.01; cfg.pso.c2 = 2.01;
cfg.pso.w0 = 0.9; cfg.pso.wf = 0.3;
cfg.pso.v0frac = 1/3;

dim = 3;            % [Kp Ki Kd]

% Limites inferiores e superiores para cada ganho
lb = [0.00  0.00  0.00];   % Kp, Ki, Kd mínimos
ub = [10.00  7.00  7.00];   % Kp, Ki, Kd máximos

% Função objetivo (handle) – cada linha x é [Kp Ki Kd]
objfunc = @(x) p3at_pid_cost(x);

% ------------------- Executa o PSO -------------------------
tic;
[gBestVal, gBestPos, gBestCurve] = pso(objfunc, dim, NoParticles, lb, ub, cfg);
t=toc;
tempo_formatado = datestr(seconds(t), 'HH:MM:SS');

fprintf('\n========= RESULTADO FINAL (PSO) =========\n');
fprintf('Kp = %.5f\n', gBestPos(1));
fprintf('Ki = %.5f\n', gBestPos(2));
fprintf('Kd = %.5f\n', gBestPos(3));
fprintf('Melhor custo J = %.6f\n', gBestVal);
disp('tempo de simulação PSO:');
disp(tempo_formatado);

% ------------------- Gráfico de convergência  --------------
figure;
plot(gBestCurve, 'LineWidth', 1.5);
grid on;
xlabel('Iteração');
ylabel('Melhor J até o momento');
title('Convergência do PSO no ajuste de [Kp Ki Kd]');
