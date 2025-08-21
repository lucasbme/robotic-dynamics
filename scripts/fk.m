%% ========================== Cinemática Direta ===========================
% Descrição: calcula cinemática direta dada a notação DH

% Tabela DH para Stanford (RRPRRR) - FALTA AJUSTAR!!!!!!!!!!!!!!!!!!
% j | theta  | d(m) | a(m) | alpha(rad)
% 1 | theta1 |  d1  |  0   | 0
% 2 | theta2 |  0   |  0   | 0
% 3 |   0    |  d3  |  0   | 0
% 4 | theta4 |  d4  |  0   | 0
% 5 | theta5 |  0   |  0   | 0
% 6 | theta6 |  d6  |  0   | 0

function [Ts] = fk(q, params)

theta1 = q(1); alpha1 = 0.0;
theta2 = q(2); alpha2 = 0.0;
d3     = q(3); alpha3 = 0.0;
theta4 = q(4); alpha4 = 0.0;
theta5 = q(5); alpha5 = 0.0;
theta6 = q(6); alpha6 = 0.0;

A1 = dh_notation(0, alpha1, params.d1, theta1);
A2 = dh_notation(0, alpha2,       0  , theta2);
A3 = dh_notation(0, alpha3,       d3 ,      0);       
A4 = dh_notation(0, alpha4, params.d4, theta4);
A5 = dh_notation(0, alpha5,      0   , theta5);
A6 = dh_notation(0, alpha6, params.d6, theta6);

Ts = cell(6,1);
Ts{1} = A1;
Ts{2} = Ts{1} * A2;
Ts{3} = Ts{2} * A3;
Ts{4} = Ts{3} * A4;
Ts{5} = Ts{4} * A5;
Ts{6} = Ts{5} * A6;

end