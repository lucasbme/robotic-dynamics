%% ========================= Cinemática Direta ==========================
% Autores: Lucas Bosso de Mello 
% Descrição: calcula cinemática direta dada a notação DH
% Útima Modificação: Lucas Bosso de Mello (18/09/2025)

%% ===== Tabela DH para Manipulador (RRPRR) ========== 
% j | theta  | d(m) | a(m) | alpha(rad)
% 1 | theta1 |  0   |  0   | -pi/2
% 2 | theta2 |  d2  |  0   |  pi/2
% 3 |   0    |  q3  |  0   | 0
% 4 | theta4 |  d4  |  0   | -pi/2
% 5 | theta5 |  0   |  0   |  pi/2

%% ========== Saídas da Função ========== 
% Tf -> Transformação Final (matriz homogênea)
% Tp -> Transformações Parciais (cada junta)

%% ========== Entradas da Função ========== 
% q -> Vetor de Estados
% params -> Parâmetros do Manipulador

function [Tf,Tp] = fk(q, params)

% ========== Inicializações ========== 
q1 = q(1); q2 = q(2); q3 = q(3); 
q4 = q(4); q5 = q(5); 

alpha1 = -pi/2; alpha2 = pi/2; alpha3 = 0.0;
alpha4 = -pi/2; alpha5 = pi/2; 

% ========== Aplica DH ========== 
A1 = dh_notation(0, alpha1,             0, q1);
A2 = dh_notation(0, alpha2,     params.d2, q2);
A3 = dh_notation(0, alpha3,            q3,  0);       
A4 = dh_notation(0, alpha4,     params.d4, q4);
A5 = dh_notation(0, alpha5,             0, q5);

% ========== Calcula Tp ========== 
Tp = cell(5,1);

Tp{1} = A1;
Tp{2} = Tp{1} * A2;
Tp{3} = Tp{2} * A3;
Tp{4} = Tp{3} * A4;
Tp{5} = Tp{4} * A5;

% ==========  Extrai Tf ========== 
Tf = Tp{5};

end
