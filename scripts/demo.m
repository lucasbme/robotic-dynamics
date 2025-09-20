%% ============= Controle Principal ============= 
% Autores: Lucas Bosso de Mello
% Descrição: Controle principal do programa
% Data: 21/08/2025
% Útima Modificação: Lucas Bosso de Mello (18/09/2025)

clear; close; clc;

%% ===================== Parâmetros =====================
% ====================== Distâncias ====================== 

params.d2 = 0.20;   % Distância entre as juntas 1 e 2 [m]
params.d3 = 0.20;   % Comprimento da prismática [m]
params.d4 = 0.20;   % Distância entre as juntas 3 e 4 [m]
params.d5 = 0.00;   % Comprimento do end-effector [m]  

%% =================== Vetor de Estados ==================
% ========== Cinemática Direta ========== 

params.q1 = deg2rad(0); % Rotação junta 1
params.q2 = deg2rad(0); % Rotação junta 2
params.q4 = deg2rad(0); % Rotação junta 4
params.q5 = deg2rad(0); % Rotação junta 5

% ===== Cria o VE =====
q = [params.q1  params.q2   params.d3  params.q4   params.q5];

%% ==================== Implementação ====================
% ========== Cinemática Direta ========== 
% Tp -> Transformações Parciais (cada junta)
% Tf -> Transformação Final (Matriz homogênea)
[Tf,Tp] = fk(q, params);
disp('Tf:'); disp(Tf);

plot3d(Tp, 1);

% ========== Cinemática Inversa ========== 
q_des = [0.2, 0.0, 0.3];
[q_sol, Tf_sol, Ts_sol] = ik(q_des, params);
disp('Solutions: '); disp(q_sol);

% ========== Jacobiano ========== 
J = jacobian(q, params);
disp('Jacobiano: '); disp(J);

plot3d(Ts_sol, 2);
