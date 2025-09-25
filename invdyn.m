function [Fjoints, torques] = invdyn(Habs, Labs, J, Fext, Hg, L, Q, Qd, Qdd, numlinks)
    % Function for the inverse dynamics of an open-chain robotic manipulator
    %
    % INPUT:
    % Mabs - Transformation matrix of the absolute reference frame for each link
    % Habs - Absolute forces and moments acting on each link
    % Labs - Position of the next joint for each link
    % J - Pseudo inertia tensor relative to the global reference frame
    % Fext - External forces applied on the end effector (gripper)
    %
    % OUTPUT:
    % Fjoints - Forces and moments acting on each joint
    % torques - Torque exerted by each joint

    % Initialization
    Fig(:,:,numlinks) = zeros(4,4); % Initial forces and moments at the gripper

    % External forces applied to the gripper
    Fjoints(:,:,numlinks+1) = Fext;
    torques = zeros(1,numlinks);

    % Backward recursion to compute joint reactions
    for j = numlinks:-1:3  % Iterate from the last link backward
        
        % Compute inertial forces and the weight of the link
        Fig(:,:,j) = (Habs(:,:,j+1) - Hg) * J(:,:,j) - ((Habs(:,:,j+1) - Hg) * J(:,:,j))';
        
        % Compute the forces acting on joint j
        Fjoints(:,:,j) = Fig(:,:,j) + Fjoints(:,:,j+1);
        
        % Compute the resultant force at joint j using the pseudot function
        torques(j) = pseudot(Fjoints(:,:,j), Labs(:,:,j));
    end

    % Manual calculation of first actuator dynamics

    % Extended mass matrix
    M = diag([0, 0, J(end,end,1), J(end,end,1), 0, 0, J(end,end,2), J(end,end,2), 0, 0, J(end,end,3), J(end,end,3), J(end,end,4), J(end,end,4), J(end,end,4), 0, 0]);

    % Extended jacobian
    Je = fourbarjacext([L(1),L(2),L(3)],Q(1));
    Jed = fourbarjacdext([L(1),L(2),L(3)],Q(1),Qd(1));
    
    % Weight force
    g = norm(Hg);
    G = g*[0 0 1 0 0 0 1 0 0 0 1 0 1 0 0 0 0]';
    
    % External forces matrix (last three elements are c4z, f4x, f4y)
    Fe = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -Fjoints(1,end,4) -Fjoints(2,end,4)]';

    % Last motor torque
    Fq = Je'*M*Je*Qdd(1)+Je'*M*Jed*Qd(1)-Je'*(Fe-M*G);
    
    % Compute motor torques
    torques = [Fq, torques(4), torques(5)];

end
