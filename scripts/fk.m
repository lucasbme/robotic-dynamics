%% ========================= Cinemática Direta ==========================
% Descrição: calcula cinemática direta dada a notação DH

% Tabela DH para Stanford (RRPRRR)
% j | theta  | d(m) | a(m) | alpha(rad)
% 1 | theta1 |  0   |  0   | -pi/2
% 2 | theta2 |  d2  |  0   |  pi/2
% 3 |   0    |  q3  |  0   | 0
% 4 | theta4 |  d4  |  0   | -pi/2
% 5 | theta5 |  0   |  0   |  pi/2
% 6 | theta6 |  d6  |  0   | 0

function [Tf,Ts] = fk(q, params)

q1 = q(1); q2 = q(2); q3 = q(3); 
q4 = q(4); q5 = q(5); q6 = q(6);

alpha1 = -pi/2; alpha2 = pi/2; alpha3 = 0.0;
alpha4 = -pi/2; alpha5 = pi/2; alpha6 = 0.0;

A1 = dh_notation(0, alpha1,             0, q1);
A2 = dh_notation(0, alpha2,     params.d2, q2);
A3 = dh_notation(0, alpha3,            q3,  0);       
A4 = dh_notation(0, alpha4,     params.d4, q4);
A5 = dh_notation(0, alpha5,             0, q5);
A6 = dh_notation(0, alpha6,     params.d6, q6);

Ts = cell(6,1);

Ts{1} = A1;
Ts{2} = Ts{1} * A2;
Ts{3} = Ts{2} * A3;
Ts{4} = Ts{3} * A4;
Ts{5} = Ts{4} * A5;
Ts{6} = Ts{5} * A6;

Tf = Ts{6};

end
