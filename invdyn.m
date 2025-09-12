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
    l1dd = L(2);   % l2 = l1
    g1dd = L(2)/2; % link are uniform bars
    g2d = L(2)/2;
    theta2  = Q(1);

    % Extended mass matrix
    M = diag([0, 0, J(end,end,3), J(end,end,3), J(end,end,1), J(end,end,1), J(end,end,2), J(end,end,3)]);

    c = cos(theta2);
    s = sin(theta2);

    % Extended jacobian
    Je = [-l1dd*s;
        l1dd*c;
        -l1dd*s;
        l1dd*c;
        -g2d*s;
        g2d*c;
        -g1dd*s;
        g1dd*c];

    Jed = [-l1dd*c;
        -l1dd*s;
        -l1dd*c;
        -l1dd*s;
        -g2d*c;
        -g2d*s;
        -g1dd*c;
        -g1dd*s]*Qd(1);

    % External forces matrix
    g = norm(Hg);
    
    Fe = [ -Fjoints(1,end,4);  % f4x
           -Fjoints(2,end,4);  % f4y
           -J(end,end,3)*g; % -m3g
           0;
           -J(end,end,2)*g; % -m2g
           0;
           0;% -J(end,end,1)*g; % -m1g
           0];
    % Last motor torque
    Fq = Je'*M*Je*Qdd(1)+Je'*M*Jed*Qd(1)-Je'*Fe;
    
    % Compute motor torques
    torques = [Fq, torques(4), torques(5)];

end
