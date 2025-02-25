function [Phistar, phi] = invdyn(Mabs, Habs, Labs, J, Phie, L)
    % Function for the inverse dynamics of an open-chain robotic manipulator
    %
    % INPUT:
    % Mabs - Transformation matrix of the absolute reference frame for each link
    % Habs - Absolute forces and moments acting on each link
    % Labs - Position of the next joint for each link
    % J - Pseudo inertia tensor relative to the local reference frame of each link
    % Phie - External forces applied on the end effector (gripper)
    %
    % OUTPUT:
    % Phistar - Forces and moments acting on each joint
    % phi - Torque exerted by each joint
    %
    % SEE ALSO:
    % pseudoinertia - Function that computes the pseudo-inertia tensor

    % Initialization
    numLinks = size(J, 3);
    Phi(:,:,numLinks) = zeros(4,4); % Initial forces and moments at the gripper
    Hg = [0, 0, 0, -9.81;  % Gravitational force applied
          0, 0, 0, 0;
          0, 0, 0, 0;
          0, 0, 0, 0];
    
    % External forces applied to the gripper
    Phistar(:,:,numLinks+1) = Phie;
    phi = zeros(1,numLinks);

    % Backward recursion to compute joint reactions
    for j = numLinks:-1:1  % Iterate from the last link backward

        % Compute pseudo-inertia tensors for each link
        % pseudoJ = pseudoinertia(I(:,:,j), m(j), G(:,:,j));
        
        % Transform the pseudo-inertia tensors from the local frame to 
        % the absolute reference frame
        % pseudoJ = Mabs(:,:,j) * J(:,:,j) * Mabs(:,:,j)';
        pseudoJ = J(:,:,j);
        
        % Compute inertial forces and the weight of the link
        Phi(:,:,j) = -((Habs(:,:,j) - Hg) * pseudoJ - ((Habs(:,:,j) - Hg) * pseudoJ)');
        
        % Compute the forces acting on joint j
        Phistar(:,:,j) = Phi(:,:,j) + Phistar(:,:,j+1);
        
        % Compute the resultant force at joint j using the pseudot function
        phi(j) = pseudot(-Phistar(:,:,j), Labs(:,:,j));
    end
        
    % Manual calculation of first actuator dynamics
    l3 = L(1);
    l2 = L(2);
    l3d = L(3);
    theta2  = acos(Mabs(1,1,3));

    % Inertia and gravitational forces on link l3
    fg = -9.81;
    
    % Link l3
    % pseudoJ = Mabs(:,:,4) * J(3) * Mabs(:,:,4)';
    Fi3(:,:) = -(Habs(:,:,4)*J(:,:,3)-J(:,:,3)*Habs(:,:,4));
    fix = Fi3(1,4);
    fiy = Fi3(2,4);
    ciz = Fi3(2,1);

    f4x = Phistar(1,4,4);
    f4y = Phistar(2,4,4);
    f4z = Phistar(3,4,4);
    c4x = Phistar(3,2,4);
    c4y = Phistar(1,3,4);
    c4z = Phistar(2,1,4);

    % Forces and torques acting on link l3
    f2x = fg - f4x + fix;
    f2y = -(4*c4z - 4*ciz + 2*f4y*l3 - 4*f4x*l3d + fg*l3d - 2*fiy*l3 + fix*l3d)/(4*l3);
    f2z = -f4z;
    c2x = - c4x - f4z*l3d;
    c2y = - c4y - (f4z*l3)/2;
    c2z = 0; % c1z is not constrained by joints

    Phistar(:,:,2) = [
         0   -c2z  c2y f2x;
         c2z  0   -c2x f2y;
        -c2y  c2x  0   f2z;
        -f2x -f2y -f2z 0];

    % Not needed
    % f1x = 0;
    % f1y = (4*c4z - 4*ciz - 2*f4y*l3 - 4*f4x*l3d + fg*l3d + 2*fiy*l3 + fix*l3d)/(4*l3);
    % f1z = 0;
    % c1x = 0;
    % c1y = 0;
    
    % Forces and torques acting on link l2
    f1x = fg - f2x + fix;
    f1y = fiy - f2y;
    f1z = -f2z;
    c1x = f2z*l2*sin(theta2) - c2x;
    c1y = - c2y - f2z*l2*cos(theta2);
    c1z = 0; % c1z is not constrained by joints
    Phistar(:,:,1) = [
         0   -c1z  c1y f1x;
         c1z  0   -c1x f1y;
        -c1y  c1x  0   f1z;
        -f1x -f1y -f1z 0];

    % Compute motor torques
    phi(1) = pseudot(-Phistar(:,:,1), Labs(:,:,2));
    phi(2) = pseudot(-Phistar(:,:,2), Labs(:,:,3));
    
    phi = [phi(1), phi(3), phi(4)];

end
