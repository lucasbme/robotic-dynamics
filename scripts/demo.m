%% ============= Controle Principal ============= 
% Autores: Lucas Bosso de Mello
% Descrição: controle principal do programa
% Data: 21/08/2025
% Útima Modificação: Lucas Bosso de Mello (22/08/2025)

clear; close; clc;

%% ===================== Parâmetros =====================
% ====================== Distâncias ====================== 

params.d2 = 0.20;   % Distância entre as juntas 1 e 2 [m]
params.d3 = 0.30;   % Comprimento da prismática [m]
params.d4 = 0.15;   % Distância entre as juntas 3 e 4 [m]
params.d6 = 0.15;   % Comprimento do end-effector [m]  

% ======================= Ângulos ======================= 

params.q1 = deg2rad(90);
params.q2 = deg2rad(30);
params.q4 = deg2rad(0);
params.q5 = deg2rad(0);
params.q6 = deg2rad(0);

% =================== Vetor de Estados ==================
q = [params.q1  params.q2   params.d3  params.q4   params.q5   params.q6];

%% ==================== Implementação ====================

[Tf,Ts] = fk(q, params);
disp('Tf ='); disp(Tf);

plot3d(Ts);