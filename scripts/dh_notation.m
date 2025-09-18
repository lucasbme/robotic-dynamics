%% ==================== Notação de Denavit-Hartenberg =====================
% Autores: Lucas Bosso de Mello
% Descrição: aplica a notação DH sobre os parâmetros do manipulador
% Útima Modificação: Lucas Bosso de Mello (18/09/2025)

function T = dh_notation(a, alpha, d, theta)

ca = cos(alpha); ct = cos(theta);
sa = sin(alpha); st = sin(theta);

T = [ct, -st * ca,  st * sa, a * ct;
     st,  ct * ca, -ct * sa, a * st;
      0,       sa,       ca,      d;
      0,        0,        0,      1
];

end
