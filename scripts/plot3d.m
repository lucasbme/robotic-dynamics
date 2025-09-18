%% ==================== Plot3D do Manipulador ====================
% Autores: Lucas Bosso de Mello
% Descrição: gera um plot3D das posições dos elos e juntas do 
%            manipulador
% Útima Modificação: Lucas Bosso de Mello (18/09/2025)

%% ========== Saídas da Função ========== 
% Gráfico 3D das posições de junta

%% ========== Entradas da Função ========== 
% T -> Célula com as transformações parciais (Tp)

function plot3d(T, idx)

pos = zeros(3, (length(T) + 1));
pos(:,1) = [0; 0; 0];

for i = 1:length(T)
    pos(:,i+1) = T{i}(1:3,4);
end

figure(idx)
plot3(pos(1,:), pos(2,:), pos(3,:), 'o-', 'LineWidth', 4, 'MarkerSize', 6); 

hold on;
grid on;
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Manipulador RRPRR');
hold off;

end
