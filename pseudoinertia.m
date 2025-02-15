function J = pseudoinertia(I, m, G)
    % Function to compute the pseudo-inertia tensor
    %
    % INPUT:
    % I - Inertia tensor relative to the center of mass (3x3 matrix)
    % m - Mass of the body (scalar)
    % G - Position vector of the center of mass relative to the local frame 
    %     [xg, yg, zg] (1x3 vector)
    %
    % OUTPUT:
    % J - Pseudo-inertia tensor in the local reference frame (4x4 matrix)

    % Compute the elements of the pseudo-inertia tensor
    Ixx = (-I(1,1) + I(2,2) + I(3,3)) / 2;
    Iyy = ( I(1,1) - I(2,2) + I(3,3)) / 2;
    Izz = ( I(1,1) + I(2,2) - I(3,3)) / 2;
    Ixy = -I(1,2);
    Iyz = -I(2,3);
    Izx = -I(3,1);
   
    % Construct the pseudo-inertia tensor
    J = [Ixx, Ixy, Izx, 0;
         Ixy, Iyy, Iyz, 0;
         Izx, Iyz, Izz, 0;
           0,   0,   0, m];
    
    % Transform the inertia matrix from the center of mass reference frame
    % to the local reference frame
    Mgi = eye(4,4);
    Mgi(1:3,4) = G';  % Translation component
    J = Mgi * J * Mgi';
end
