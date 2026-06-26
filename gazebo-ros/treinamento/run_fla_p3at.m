%% run_fla_p3at.m
% Script para ajustar [Kp Ki Kd] do controlador PID do Pioneer3AT
% usando o Ficks Law Algorithm (FLA) e a simulação no CoppeliaSim.

clear; clc; close all;

% ------------------- Parâmetros do FLA ---------------------
NoMolecules = 15;   % tamanho da população
MaxIt       = 30;   % número de iterações

dim = 3;            % [Kp Ki Kd]

% Limites inferiores e superiores para cada ganho
lb = [0.00  0.00  0.00];   % Kp, Ki, Kd mínimos
ub = [1.00  1.00  1.00];   % Kp, Ki, Kd máximos

% Função objetivo (handle) – cada linha x é [Kp Ki Kd]
objfunc = @(x) p3at_pid_cost(x);

% ------------------- Executa o FLA -------------------------
tic;
[Xbest, BestF, CNVG] = FLA(NoMolecules, MaxIt, lb, ub, dim, objfunc);
t=toc;
tempo_formatado = datestr(seconds(t), 'HH:MM:SS');


fprintf('\n========= RESULTADO FINAL (FLA) =========\n');
fprintf('Kp = %.5f\n', Xbest(1));
fprintf('Ki = %.5f\n', Xbest(2));
fprintf('Kd = %.5f\n', Xbest(3));
fprintf('Melhor custo J = %.6f\n', BestF);
disp('tempo de simulação FLA:');
disp(tempo_formatado);

% ------------------- Gráfico de convergência  --------------
figure;
plot(CNVG, 'LineWidth', 1.5);
grid on;
xlabel('Iteração');
ylabel('Melhor J até o momento');
title('Convergência do FLA no ajuste de [Kp Ki Kd]');
