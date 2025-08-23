%% ==================== Plot3D do Manipulador ====================
% Descrição: gera um plot3D das posições dos elos e juntas do 
%            manipulador

function plot3d(T)

pos = zeros(3, (length(T) + 1));
pos(:,1) = [0; 0; 0];

for i = 1:length(T)
    pos(:,i+1) = T{i}(1:3,4);
end

figure(1)
plot3(pos(1,:), pos(2,:), pos(3,:), 'o-', 'LineWidth', 4, 'MarkerSize', 6); 

hold on;
grid on;
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Stanford Manipulador');
hold off;

end