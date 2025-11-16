%% ========================= Jacobiano ==========================
% Autores: Lucas Bosso de Mello 
% Descrição: retorna o jacobiano geométrico do manipulador
% Útima Modificação: Lucas Bosso de Mello (20/09/2025)

%% ========== Saídas da Função ========== 
% J -> Jacobiano

%% ========== Entradas da Função ========== 
% q -> Vetor de Estados
% params -> Parâmetros do Manipulador

function J = jacobian(q, params)

% ===== FK =====
% Tp -> Transformações Parciais
% Tf -> Transformação Final
[Tf,Tp] = fk(q,params);

% ===== Posição do End-Effector =====
endPosition = Tf(1:3,4);

% ===== Número de Juntas =====
n = length(q);

% ===== Inicializações =====
Jv = zeros(3,n);    % Velocidades Lineares
Jw = zeros(3,n);    % Velocidades Angulares

% ===== Extrai z(i-1) e p(i-1) da transformação anterior =====
for i = 1:n
    if i == 1
        % ===== Frame da Base =====
        zPrev = [0;0;1]; % Eixo da junta da base
        pPrev = [0;0;0]; % Posição da junta da base
    else
        Tprev = Tp{i-1};
        zPrev = Tprev(1:3,3);
        pPrev = Tprev(1:3,4);
    end
     
    %% ===== Aplica Jacobiano =====
    % Para juntas de Revolução (q1, q2, q4, q5)
    if (i == 1) || (i == 2) || (i == 4) || (i == 5)
        Jv(:,i) = cross(zPrev, endPosition - pPrev);
        Jw(:,i) = zPrev;
    else
        % Para juntas Prismáticas (q3)
        Jv(:,i) = zPrev;
        Jw(:,i) = [0;0;0];
    end
end

J = [Jv; Jw];

end