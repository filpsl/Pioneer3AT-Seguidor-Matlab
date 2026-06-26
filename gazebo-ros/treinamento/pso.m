function [gBestVal, gBestPos, gBestCurve] = pso(f, N, S, lb, ub, cfg)
% Algoritmo PSO canônico compatível com main.m
% f   : handle da função-objetivo, recebe x(1xN)
% N   : dimensionalidade
% S   : tamanho do enxame (nº de partículas)
% lb  : limites inferiores (1xN)
% ub  : limites superiores (1xN)
% cfg : estrutura com os campos:
%       cfg.iter,
%       cfg.pso.c1, cfg.pso.c2, cfg.pso.w0, cfg.pso.wf, cfg.pso.v0frac

    % ---- parâmetros gerais a partir de cfg ----
    maxIter = cfg.iter;
    c1 = cfg.pso.c1; 
    c2 = cfg.pso.c2;
    w0 = cfg.pso.w0; 
    wf = cfg.pso.wf;
    v0frac = cfg.pso.v0frac;

    % ---- preparar vetores/limites como linha 1xN ----
    lb = lb(:)'; 
    ub = ub(:)';
    span = ub - lb;

    % Velocidade: limite clássico proporcional ao range
    vmax = span; 
    vmin = -vmax;
    % Velocidade inicial limitada por fração do range
    v0 = span * v0frac;

    % ---- inicialização ----
    % posições uniformes no hipercubo [lb,ub]
    X = lb + rand(S,N).*span;
    % velocidades iniciais em [-v0, v0]
    V = -v0 + (2*v0).*rand(S,N);

    % Avaliar população inicial
    Fx = zeros(S,1);
    for i = 1:S
        Fx(i) = feval(f, X(i,:));
    end

    % melhores pessoais (pbest) e global (gbest)
    Pbest = X;
    Fp = Fx;
    [gBestVal, idx] = min(Fx);
    gBestPos = X(idx,:);

    % Curva de melhor global por iteração
    gBestCurve = zeros(1, maxIter);

    % ---- laço principal ----
    for it = 1:maxIter
        % peso de inércia decaindo linearmente de w0 -> wf
        if maxIter > 1
            w = w0 + (wf - w0)*(it-1)/(maxIter-1);
        else
            w = wf;
        end

        % atualização das velocidades/posições
        r1 = rand(S,N);
        r2 = rand(S,N);
        V = w.*V + c1.*r1.*(Pbest - X) + c2.*r2.*(gBestPos - X);

        % clamp em velocidade
        V = max(min(V, vmax), vmin);

        % move
        X = X + V;

        % clamp em posição
        X = max(min(X, ub), lb);

        % reavalia
        for i = 1:S
            Fx(i) = feval(f, X(i,:));
        end

        % atualiza pbest
        improved = Fx < Fp;
        if any(improved)
            Pbest(improved,:) = X(improved,:);
            Fp(improved)      = Fx(improved);
        end

        % atualiza gbest
        [val, idx] = min(Fx);
        fprintf('Iteração: %f', it);
        if val < gBestVal
            gBestVal = val
            gBestPos = X(idx,:)
        end

        gBestCurve(it) = gBestVal;
    end
end
