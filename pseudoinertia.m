function J = pseudoinertia(I, m, G)
    % Funzione per calcolare lo pseudo-tensore di inerzia
    % INPUT:
    % I - Tensore di inerzia rispetto al baricentro (3x3 matrix)
    % m - Massa del corpo (scalar)
    % G - Vettore posizione del baricentro [xg, yg, zg] (1x3 vector)
    %
    % OUTPUT:
    % J - Pseudo-tensore di inerzia (4x4 matrix)

    Ixx = (-I(1,1)+I(2,2)+I(3,3))/2;
    Iyy = (+I(1,1)-I(2,2)+I(3,3))/2;
    Izz = (+I(1,1)+I(2,2)-I(3,3))/2;
    Ixy = -I(1,2);
    Iyz = -I(2,3);
    Izx = -I(3,1);

    J = [Ixx, Ixy, Izx, m*G(1);
         Ixy, Iyy, Iyz, m*G(2);
         Izx, Iyz, Izz, m*G(3);
         m*G(1), m*G(2), m*G(3),  m];
end
