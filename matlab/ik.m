%% ========================= Cinemática Inversa (entrada: estados) ==========================
% Autores: Lucas Bosso de Mello
% Descrição: resolve a cinemática inversa para o manipulador (RRPRR)
% Útima Modificação: Lucas Bosso de Mello (18/09/2025)

%% ========== Entradas da Função ========== 
%   q_des  -> vetor de estados desejado (x,y,z)
%   params -> parâmetros DH do manipulador

%% ========== Saídas da Função ========== 
%   q_sol  -> vetor de estado das juntas encontrado pela IK
%   Tf_sol -> matriz homogênea final da solução
%   Ts_sol -> células com todas as transformações parciais (Tp)

function [q_sol, Tf_sol, Ts_sol] = ik(q_des, params)

    % Extração da posição
    px = q_des(1); py = q_des(2); pz = q_des(3);

    % ---------- Resolver q1 e q2 ----------
    q1 = atan2(py, px);
    r = sqrt(px^2 + py^2);
    q2 = atan2(pz - params.d2, r);

    % ---------- Resolver q3 (prismática) ----------
    q3 = sqrt(px^2 + py^2 + (pz - params.d2)^2);

    %% ---------- Punho - DEFINIR ORIENTACAO!!!!!!!!!! ----------
    % Assumindo q4 = 0, q5 = 0
    q4 = 0;  
    q5 = 0;

    % ---------- Montar solução ----------
    q_sol = [q1, q2, q3, q4, q5];

    % ---------- Calcular FK ----------
    [Tf_sol, Ts_sol] = fk(q_sol, params);

end
