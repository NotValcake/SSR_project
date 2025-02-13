function J = pseudoinertia(I, m, G)
    % Funzione per calcolare lo pseudo-tensore di inerzia
    % INPUT:
    % I - Tensore di inerzia rispetto al baricentro (3x3 matrix)
    % m - Massa del corpo (scalar)
    % G - Vettore posizione del baricentro rispetto al sistema di 
    %     riferimento locale [xg, yg, zg] (1x3 vector)
    % OUTPUT:
    % J - Pseudo-tensore di inerzia nel sistema di riferimento
    %     locale (4x4 matrix)

    Ixx = (-I(1,1)+I(2,2)+I(3,3))/2;
    Iyy = (+I(1,1)-I(2,2)+I(3,3))/2;
    Izz = (+I(1,1)+I(2,2)-I(3,3))/2;
    Ixy = -I(1,2);
    Iyz = -I(2,3);
    Izx = -I(3,1);
   
    J = [Ixx, Ixy, Izx, 0;
         Ixy, Iyy, Iyz, 0;
         Izx, Iyz, Izz, 0;
           0,   0,   0, m];
    
    % Trasformazione della matrice di inerzia dal sistema di riferimento
    % del baricentro a quello locale
    Mgi = eye(4,4);
    Mgi(1:3,4) = G';
    J = Mgi*J*Mgi';
end
