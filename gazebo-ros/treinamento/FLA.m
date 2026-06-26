% FLA  Fick's Law Algorithm 
% -------------------------------------------------------------------------
% Implementa a meta-heurística baseada na Lei de Fick (difusão) para
% otimização numérica contínua.
%
% PARÂMETROS DE ENTRADA
%   NoMolecules : tamanho da população (número de "moléculas"/soluções)
%   MaxIt       : número máximo de iterações
%   lb, ub      : limites inferior e superior (escalar ou vetor 1xdim)
%   dim         : dimensionalidade do problema
%   objfunc     : handle da função objetivo: f = objfunc(x)
%
% SAÍDAS
%   Xbest       : melhor solução encontrada
%   BestF       : melhor valor da função objetivo
%   CNVG        : vetor de convergência (melhor custo por iteração)
% -------------------------------------------------------------------------
%
% Referência:
%   Main paper: Fatma Hashim, Reham R Mostafa, Abdelazim G. Hussien,
%               Seyedali Mirjalili, & Karam M. Sallam, Knowledge-based Systems

function [Xbest, BestF, CNVG] = FLA(NoMolecules, MaxIt, lb, ub, dim, objfunc)

% ----------------------- Tratamento de limites ---------------------------
if isscalar(lb), lb = lb*ones(1,dim); end
if isscalar(ub), ub = ub*ones(1,dim); end

% ----------------------- Hiperparâmetros originais ----------------------
C1 = 0.5;    % controla a função de transferência TF(t) = sinh(t/T)^C1
C2 = 2;      % usado no cálculo de DOF (intensidade de exploração)
c3 = 0.1;    % fração mínima de moléculas transferidas (DO)
c4 = 0.2;    % fração máxima de moléculas transferidas (DO)
c5 = 2;      % termo linear em TF para direção de fluxo (TDO)
D  = 0.01;   % coeficiente de difusão (Lei de Fick) - intensidade do fluxo

% ----------------------- Inicialização da população ----------------------
% Posições aleatórias uniformes em [lb, ub]
X = lb + rand(NoMolecules,dim).* (ub - lb);

% Avaliação inicial
FS = arrayfun(@(i) feval(objfunc, X(i,:)), 1:NoMolecules);
[BestF, idxBest] = min(FS);
Xbest = X(idxBest,:);

% Divisão da população em dois grupos (regiões): N1 e N2
n1 = round(NoMolecules/2);
n2 = NoMolecules - n1;
X1 = X(1:n1,:);
X2 = X(n1+1:end,:);

FS1 = arrayfun(@(i) feval(objfunc, X1(i,:)), 1:n1);
FS2 = arrayfun(@(i) feval(objfunc, X2(i,:)), 1:n2);

% Melhores locais (equilíbrio) de cada grupo
[FS_eo1, idx_eo1] = min(FS1);  Xeo1 = X1(idx_eo1,:);
[FS_eo2, idx_eo2] = min(FS2);  Xeo2 = X2(idx_eo2,:);

% Melhor entre os grupos, usado como "estado estacionário" de referência
if FS_eo1 < FS_eo2
    FSss = FS_eo1; YSol = Xeo1;
else
    FSss = FS_eo2; YSol = Xeo2;
end

% Utilitários
vec_flag = [1, -1];         % direção do fluxo (sinal)
CNVG = zeros(1, MaxIt);     % histórico de melhor custo
eps_num = eps;              % pequena constante para evitar divisão por zero

% ----------------------- Laço principal ----------------------------------
for t = 1:MaxIt
    
    % Função de transferência TF(t) define o estágio (DO/EO/SSO).
    TF = sinh(t/MaxIt)^C1;

    % Junta temporariamente os grupos para algumas estatísticas globais
    X = [X1; X2];

    % ======================= (1) DO - Diffusion Operator ==================
    if TF < 0.9
        % Grau de exploração (diretamente dependente de TF e C2)
        DOF = exp(-(C2*TF - rand()))^C2;

        % Direção de fluxo TDO: se < rand => Transferência entre regiões
        TDO = c5*TF - rand();

        if (TDO) < rand()
            % Número de moléculas que viajam de uma região para outra
            M1N = c3*n1;          % min
            M2N = c4*n1;          % max
            NT12 = round((M2N - M1N).*rand() + M1N);

            % --- Atualiza parte de X1 indo em direção à Xeo2 (fluxo 1 -> 2)
            X1new = X1;  % pré-aloca
            for u = 1:NT12
                DFg = vec_flag( floor(2*rand()) + 1 ); % +1: índice 1 ou 2
                Xm2 = mean(X2,1);
                Xm1 = mean(X1,1);
                % Lei de Fick (discretizada) para estimar o "fluxo" J
                J = -D * (Xm2 - Xm1) / ( norm(Xeo2 - X1(u,:), 2) + eps_num );
                % Passo de movimento com ruído e direção de fluxo
                X1new(u,:) = Xeo2 + DFg*DOF.*rand(1,dim).*( J.*Xeo2 - X1(u,:) );
            end

            % --- Restante de X1 (operações locais/aleatórias)
            for u = NT12+1:n1
                for tt = 1:dim
                    p = rand();
                    if p < 0.8
                        X1new(u,tt) = Xeo1(tt);
                    elseif p < 0.9
                        r3 = rand();
                        X1new(u,tt) = X1(u,tt) + DOF.*((ub(tt)-lb(tt))*r3 + lb(tt));
                    else
                        X1new(u,tt) = X1(u,tt);
                    end
                end
            end

            % --- Atualiza X2 (movimento em direção à Xeo2 com ruído)
            X2new = X2; % pré-aloca
            for u = 1:n2
                r4 = rand();
                X2new(u,:) = Xeo2 + DOF.*( (ub - lb)*r4 + lb );
            end

        else
            % Caso contrário, transferência oposta (2 -> 1) com frações de n2
            M1N = 0.1*n2;
            M2N = 0.2*n2;
            Ntransfer = round((M2N - M1N).*rand() + M1N);

            X2new = X2; X1new = X1; % pré-aloca

            for u = 1:Ntransfer
                DFg = vec_flag( floor(2*rand()) + 1 );
                Xm1 = mean(X1,1);
                Xm2 = mean(X2,1);
                J   = -D * (Xm1 - Xm2) / ( norm(Xeo1 - X2(u,:), 2) + eps_num );
                X2new(u,:) = Xeo1 + DFg*DOF.*rand(1,dim).*( J.*Xeo1 - X2(u,:) );
            end

            for u = Ntransfer+1:n2
                for tt = 1:dim
                    p = rand();
                    if p < 0.8
                        X2new(u,tt) = Xeo2(tt);
                    elseif p < 0.9
                        r3 = rand();
                        X2new(u,tt) = X2(u,tt) + DOF.*((ub(tt)-lb(tt))*r3 + lb(tt));
                    else
                        X2new(u,tt) = X2(u,tt);
                    end
                end
            end

            for u = 1:n1
                r4 = rand();
                X1new(u,:) = Xeo1 + DOF.*( (ub - lb)*r4 + lb );
            end
        end

    % ======================= (2) EO - Equilibrium Operator =================
    elseif TF <= 1
        X1new = X1; X2new = X2; % pré-aloca

        % Grupo 1
        for u = 1:n1
            DFg  = vec_flag( floor(2*rand()) + 1 );
            Xm1  = mean(X1,1);
            J    = -D * (Xeo1 - Xm1) / ( norm(Xeo1 - X1(u,:), 2) + eps_num );
            DRF  = exp(-J/TF);                         % Diffusion Rate Factor
            MS   = exp(-FS_eo1/(FS1(u) + eps_num));    % Motion Step
            R1   = rand(1,dim);
            Qeo  = DFg * DRF .* R1;
            X1new(u,:) = Xeo1 + Qeo.*X1(u,:) + Qeo.*(MS*Xeo1 - X1(u,:));
        end

        % Grupo 2
        for u = 1:n2
            DFg  = vec_flag( floor(2*rand()) + 1 );
            Xm2  = mean(X2,1);
            J    = -D * (Xeo2 - Xm2) / ( norm(Xeo2 - X2(u,:), 2) + eps_num );
            DRF  = exp(-J/TF);
            MS   = exp(-FS_eo2/(FS2(u) + eps_num));
            R1   = rand(1,dim);
            Qeo  = DFg * DRF .* R1;
            % OBS: o código original usa Xeo1 aqui; mantemos por fidelidade.
            X2new(u,:) = Xeo2 + Qeo.*X2(u,:) + Qeo.*(MS*Xeo1 - X2(u,:));
        end

    % ======================= (3) SSO - Steady-State Operator ==============
    else
        X1new = X1; X2new = X2; % pré-aloca

        % Grupo 1
        for u = 1:n1
            DFg  = vec_flag( floor(2*rand()) + 1 );
            Xm1  = mean(X1,1);
            Xm   = mean(X,1);
            J    = -D * (Xm - Xm1) / ( norm(Xbest - X1(u,:), 2) + eps_num );
            DRF  = exp(-J/TF);
            MS   = exp(-FSss/(FS1(u) + eps_num));
            R1   = rand(1,dim);
            Qg   = DFg * DRF .* R1;
            X1new(u,:) = Xbest + Qg.*X1(u,:) + Qg.*(MS*Xbest - X1(u,:));
        end

        % Grupo 2
        for u = 1:n2
            DFg  = vec_flag( floor(2*rand()) + 1 );
            Xm1  = mean(X1,1);
            Xm   = mean(X,1);
            J    = -D * (Xm1 - Xm) / ( norm(Xbest - X2(u,:), 2) + eps_num );
            DRF  = exp(-J/TF);
            MS   = exp(-FSss/(FS2(u) + eps_num));
            R1   = rand(1,dim);              % garante R1 válido
            Qg   = DFg * DRF .* R1;
            X2new(u,:) = Xbest + Qg.*X2(u,:) + Qg.*(MS*Xbest - X2(u,:));
        end
    end

    % ----------------------- Correção de limites e seleção -----------------
    for j = 1:n1
        X1new(j,:) = min(max(X1new(j,:), lb), ub);
        v = feval(objfunc, X1new(j,:));
        if v < FS1(j)
            FS1(j) = v;
            X1(j,:) = X1new(j,:);
        end
    end

    for j = 1:n2
        X2new(j,:) = min(max(X2new(j,:), lb), ub);
        v = feval(objfunc, X2new(j,:));
        if v < FS2(j)
            FS2(j) = v;
            X2(j,:) = X2new(j,:);
        end
    end

    % ----------------------- Atualiza melhores por grupo -------------------
    [FS_eo1, idx_eo1] = min(FS1);  Xeo1 = X1(idx_eo1,:);
    [FS_eo2, idx_eo2] = min(FS2);  Xeo2 = X2(idx_eo2,:);

    % Melhor entre os grupos (estado estacionário de referência)
    if FS_eo1 < FS_eo2
        FSss = FS_eo1;  YSol = Xeo1;
    else
        FSss = FS_eo2;  YSol = Xeo2;
    end

    % Histórico de convergência e melhor global
    t
    CNVG(t) = FSss;
    if FSss < BestF
        BestF = FSss
        Xbest = YSol
    end
end
end


% %_________________________________________________________________________%
% %  Fick's Law Algorithm (FLA) source codes version 1.0                    %
% %                                                                         %
% %  Developed in MATLAB R2021b                                             %
% %                                                                         %
% %  Coresponding Author:  Abdelazim G. Hussien                             %
% %                                                                         %
% %                                                                         %
% %         e-Mail: abdelazim.hussien@liu.se                                %
% %                 aga08@fayoum.edu.eg                                     %
% %                                                                         %
% %                                                                         %
% %   Main paper: Fatma Hashim, Reham R Mostafa, Abdelazim G. Hussien,      %
% %                     Seyedali Mirjalili, & Karam M. Sallam               %
% %               Knowledge-based Systems                                   %
% %                                                                         %
% %_________________________________________________________________________%
% function [Xss, BestF, CNVG] = FLA( NoMolecules, T,lb,ub, dim,objfunc)
% C1=0.5;C2=2;c3=.1;c4=.2;c5=2;
% D=.01;
% X=lb+rand(NoMolecules,dim)*(ub-lb);%intial positions
% for i=1:NoMolecules
% FS(i) = feval(objfunc,X(i,:));
% end
% [BestF, IndexBestF] = min(FS);
% Xss = X(IndexBestF,:);
% n1=round(NoMolecules/2);
% n2=NoMolecules-n1;
% X1=X(1:n1,:);
% X2=X(n1+1:NoMolecules,:);
% for i=1:n1
% FS1(i) = feval(objfunc,X1(i,:));
% end
% for i=1:n2
% FS2(i) = feval(objfunc,X2(i,:));
% end
% 
% [FSeo1, IndexFSeo1] = min(FS1);
% [FSeo2, IndexFSeo2] = min(FS2);
% Xeo1 = X1(IndexFSeo1,:);
% Xeo2 = X2(IndexFSeo2,:);
% vec_flag=[1,-1];
%   if FSeo1<FSeo2
%         FSss=FSeo1;
%         YSol=Xeo1;
%     else
%         FSss=FSeo2;
%         YSol=Xeo2;
%     end
% for t = 1:T
%     TF(t)=sinh(t/T)^C1;
%     X=[X1;X2];
%     %             Difusion Operator
%     if TF(t)<0.9
%            DOF=exp(-(C2*TF(t)-rand))^C2;
%             TDO=c5*TF(t)-rand;%   direction of flow
%             if (TDO)<rand
%                 %         select no of molecules
%                 M1N = c3*n1;
%                 M2N = c4*n1;
%                 NT12 =round((M2N-M1N).*rand(1,1) + M1N);
%                 for u=1:NT12
%                     flag_index = floor(2*rand()+1);
%                     DFg=vec_flag(flag_index);
%                     Xm2=mean(X2);
%                     Xm1=mean(X1);
%                     J=-D*(Xm2-Xm1)/norm(Xeo2- X1(u,:)+eps);
%                     X1new(u,:)= Xeo2+ DFg*DOF.*rand(1,dim).*(J.*Xeo2-X1(u,:));
%                 end
%                 for u=NT12+1:n1
%                     for tt=1:dim
%                         p=rand;
%                         if p<0.8
%                             X1new(u,tt) = Xeo1(tt);
%                         elseif p<.9
%                             r3=rand;
%                             X1new(u,tt)=X1(u,tt)+DOF.*((ub-lb)*r3+lb);
%                         else
%                             X1new(u,tt) =X1(u,tt);
%                         end
% 
%                     end
%                 end
%                 for u=1:n2
%                     r4=rand;
%                     X2new(u,:)= Xeo2+DOF.*((ub-lb)*r4+lb);
%                 end
%             else
%                 M1N = .1*n2;
%                 M2N = .2*n2;
%                 Ntransfer =round((M2N-M1N).*rand(1,1) + M1N);
%                 for u=1:Ntransfer
%                     flag_index = floor(2*rand()+1);
%                     DFg=vec_flag(flag_index);
%                     R1=randi(n1);
%                     Xm1=mean(X1);
%                     Xm2=mean(X2);
%                     J=-D*(Xm1-Xm2)/norm(Xeo1- X2(u,:)+eps);
%                     X2new(u,:)=  Xeo1+DFg*DOF.*rand(1,dim).*(J.*Xeo1-1*X2(u,:));
%                 end
%                 for u=Ntransfer+1:n2
%                     for tt=1:dim
%                         p=rand;
%                         if p<0.8
%                             X2new(u,tt) = Xeo2(tt);
%                         elseif p<.9
%                             r3=rand;
%                             X2new(u,tt)=X2(u,tt)+DOF.*((ub-lb)*r3+lb);
%                         else
%                             X2new(u,tt) =X2(u,tt);
%                         end
% 
%                     end
%                 end
%                 for u=1:n1
%                     r4=rand;
%                     X1new(u,:)= Xeo1+DOF.*((ub-lb)*r4+lb);
%                 end
%             end
% 
%     else
% %         Equilibrium operator (EO)
%         if TF(t)<=1
%             for u=1:n1
%                 flag_index = floor(2*rand()+1);
%                 DFg=vec_flag(flag_index);
%                 Xm1=mean(X1);
%                 Xmeo1=Xeo1;
%                 J=-D*(Xmeo1-Xm1)/norm(Xeo1- X1(u,:)+eps);
%                 DRF= exp(-J/TF(t));
%                 MS=exp(-FSeo1/(FS1(u)+eps));
%                 R1=rand(1,dim);
%                 Qeo=DFg*DRF.*R1;
%                 X1new(u,:)= Xeo1+Qeo.*X1(u,:)+Qeo.*(MS*Xeo1-X1(u,:));
%             end
%             for u=1:n2
%                 flag_index = floor(2*rand()+1);
%                 DFg=vec_flag(flag_index);
%                 Xm2=mean(X2);
%                 Xmeo2=Xeo2;
%                 J=-D*(Xmeo2-Xm2)/norm(Xeo2- X2(u,:)+eps);
%                 DRF= exp(-J/TF(t));
%                 MS=exp(-FSeo2/(FS2(u)+eps));
%                 R1=rand(1,dim);
%                 Qeo=DFg*DRF.*R1;
%                 X2new(u,:)=  Xeo2+Qeo.*X2(u,:)+Qeo.*(MS*Xeo1-X2(u,:));
%             end
%         else
%             %     Steady state operator (SSO):
%                 for u=1:n1
%             flag_index = floor(2*rand()+1);
%             DFg=vec_flag(flag_index);
%             Xm1=mean(X1);
%             Xm=mean(X);
%             J=-D*(Xm-Xm1)/norm(Xss- X1(u,:)+eps);
%             DRF= exp(-J/TF(t));
%             MS=exp(-FSss/(FS1(u)+eps));
%             R1=rand(1,dim);
%             Qg=DFg*DRF.*R1;
%             X1new(u,:)=  Xss+Qg.*X1(u,:)+Qg.*(MS*Xss-X1(u,:));
%         end
%         for u=1:n2
%             Xm1=mean(X1);
%             Xm=mean(X);
%             J=-D*(Xm1-Xm)/norm(Xss- X2(u,:)+eps);
%             DRF= exp(-J/TF(t));
%             MS=exp(-FSss/(FS2(u)+eps));
%             flag_index = floor(2*rand()+1);
%             DFg=vec_flag(flag_index);
%                         Qg=DFg*DRF.*R1;
%             X2new(u,:)= Xss+ Qg.*X2(u,:)+Qg.*(MS*Xss-X2(u,:));
%         end
%         end
%     end
%     for j=1:n1
%         FU=X1new(j,:)>ub;FL=X1new(j,:)<lb;X1new(j,:)=(X1new(j,:).*(~(FU+FL)))+ub.*FU+lb.*FL;
%         v = feval(objfunc,X1new(j,:));
%         if v<FS1(j)
%             FS1(j)=v;
%             X1(j,:)= X1new(j,:);
%         end
%     end
%     for j=1:n2
%         FU=X2new(j,:)>ub;FL=X2new(j,:)<lb;X2new(j,:)=(X2new(j,:).*(~(FU+FL)))+ub.*FU+lb.*FL;
%         v = feval(objfunc,X2new(j,:));
%         if v<FS2(j)
%             FS2(j)=v;
%             X2(j,:)= X2new(j,:);
%         end
%     end
% 
%     [FSeo1, IndexFSeo1] = min(FS1);
%     [FSeo2, IndexFSeo2] = min(FS2);
% 
%     Xeo1 = X1(IndexFSeo1,:);
%     Xeo2 = X2(IndexFSeo2,:);
%     if FSeo1<FSeo2
%         FSss=FSeo1;
%         YSol=Xeo1;
%     else
%         FSss=FSeo2;
%         YSol=Xeo2;
%     end
%     CNVG(t)=FSss;
%     if FSss<BestF
%         BestF=FSss;
%         Xss =YSol;
% 
%     end
%     %disp(['Iteration ' num2str(t) ':   ' ...
%     %    'Best Cost Bw = ' num2str(BestF)]);  
% end
% 
% end