function [Mabs, Wabs, Habs, Labs] = dirkin(L, Q, Qd, Qdd)
% DIRKIN - Computes the direct kinematics of a manipulator.
%
% This function calculates the position, velocity, and acceleration of the
% end-effector (gripper) of a manipulator based on the dimensions and positions
% of its actuators. It takes the link lengths and joint positions, velocities, 
% and accelerations, outputting the transformation matrix of the end-effector 
% along with its velocity and acceleration in the workspace.
%
% Syntax:
%   [P, Pd, Pdd] = dirkin(L, Q, Qd, Qdd)
%
% Inputs:
%   L - Vector of link lengths of the manipulator, defined as:
%       L(1): Length of link 1
%       L(2): Length of link 2
%       L(3): Length of link 3
%       L(4): Length of link 4
%       L(5): Length of link 5
%
%   Q - Vector of joint positions (angles in radians) for each actuated joint of the 
%       manipulator.
%
%   Qd - Vector of joint velocities for each actuated joint.
%
%   Qdd - Vector of joint accelerations for each actuated joint.
%
% Outputs:
%   M     - 4x4 transformation matrices representing the position and orientation 
%           of each joint in the global coordinate frame.
%   Wabs  - 4x4 matrix representing the velocity of each joint. In order to get
%           the actual velocity it shall be multiplied by the respective M.
%   Habs  - 4x4 matrix representing the acceleration of each joint. In order to get
%           the actual acceleration it shall be multiplied by the respective M.
%   Labs  - 4x4 matrix representing the kinematic of each joint.
%
% Details:
%   The function constructs the transformation matrices for each link based on
%   given angles and lengths. Each joint's transformation is determined using 
%   predefined angles (`theta2i`, `theta3i`, etc.) and applied rotations around
%   the Z and Y axes based on `Q`, `Qd`, and `Qdd`.
%
%   - The matrices `Lrz` and `Lry` represent rotational transformations around
%     the Z and Y axes, respectively. These are used to compute the angular 
%     velocities and accelerations for each joint.
%   - Jacobian-based transformations for the angular velocities (`Wabs`) 
%     and accelerations (`Habs`) are updated iteratively for each link based on
%     recursive kinematic relationships. Special cases for the parallelogram 
%     structure are handled when updating transformations for `M(:,:,7)` and 
%     `M(:,:,8)`.
%
% Computation:
%   - Transformation matrices `M` are created for each segment based on the 
%     given configuration, where each matrix corresponds to the link's 
%     position and orientation.
%   - For each joint, the velocity (`Pd`) and acceleration (`Pdd`) matrices 
%     are calculated in the end-effector frame. They are obtained by 
%     combining the relative and absolute angular velocity/acceleration matrices
%     with the end-effector's transformation matrix.
%
% Example:
%   % Define the lengths of each link
%   L = [0.5, 0.3, 0.4, 0.2, 0.1];
%
%   % Define the joint configurations (angles in radians, velocities, and accelerations)
%   Q = [pi/4, pi/6, pi/3];
%   Qd = [0.1, 0.2, 0.3];
%   Qdd = [0.01, 0.02, 0.03];
%
%   % Calculate position, velocity, and acceleration matrices for each joint
%   [M, Wabs, Habs] = dirkin(L, Q, Qd, Qdd);
%
%   % Calculate gripper position, velocity and acceleration
%   P = M(:,:,6);
%   Pd = Wabs(:,:,6)*P;
%   Pdd = Habs(:,:,6)*P;
%
% Notes:
%   This function assumes a specific parallelogram configuration in the 
%   manipulator's design, and the transformation matrix construction accounts 
%   for the fixed angles and joint limits. Make sure to provide the correct 
%   input dimensions for L, Q, Qd, and Qdd.
%
% See also: posmat

l1i  = L(1);
l2i  = L(2);
l3ii = L(3);
l1ii = l2i;
l3i  = l1i/2;
l4   = L(4);
l5   = L(5);

theta2i  = Q(1);
theta2id = Qd(1);
theta2idd = Qdd(1);

theta3i  = pi-theta2i;
theta1ii = theta2i;
theta3ii = theta3i+pi;

theta4   = Q(2);
theta4d = Qd(2);
theta4dd = Qdd(2);

theta5   = Q(3);
theta5d = Qd(3);
theta5dd = Qdd(3);

% Reordering the Qd and Qdd matrices for iterative solution
Qd(1) = 0; 
Qd(2) = theta2id; Qd(3) = -theta2id;
Qd(4) = theta4d;
Qd(5) = theta5d;
Qd(6) = theta2id; Qd(7) = -theta2id;

Qdd(1) = 0; 
Qdd(2) = theta2idd; Qdd(3) = -theta2idd;
Qdd(4) = theta4dd;
Qdd(5) = theta5dd;
Qdd(6) = theta2idd; Qdd(7) = -theta2idd;


M(:,:,1) = eye(4,4);
M(:,:,2) = posmat(0, 'z', l1i);
M(:,:,3) = posmat(theta2i, 'z', l2i);
M(:,:,4) = [cos(theta3i-pi/2), -sin(theta3i-pi/2), 0, l3i*cos(theta3i)+l3ii*cos(theta3i-pi/2);
         sin(theta3i-pi/2), cos(theta3i-pi/2), 0, l3i*sin(theta3i)+l3ii*sin(theta3i-pi/2);
         0, 0, 1, 0;
         0, 0, 0, 1];
M(:,:,5) = posmat(theta4, 'y', l4);
M(:,:,6) = posmat(theta5, 'z', l5);
M(:,:,7) = posmat(theta1ii, 'z', l1ii);
M(:,:,8) = [cos(theta3ii+pi/2), -sin(theta3ii+pi/2), 0, l3i*cos(theta3ii)+l3ii*cos(theta3ii+pi/2);
         sin(theta3ii+pi/2), cos(theta3ii+pi/2), 0, l3i*sin(theta3ii)+l3ii*sin(theta3ii+pi/2);
         0, 0, 1, 0;
         0, 0, 0, 1];

Lrz = [0, -1, 0, 0;
       1, 0, 0, 0;
       0, 0, 0, 0;
       0, 0, 0, 0];
Lry = [0, 0, 1, 0;
       0, 0, 0, 0;
       -1, 0, 0, 0;
       0, 0, 0, 0];

Wabs(:,:,1) = zeros(4,4);
Habs(:,:,1) = zeros(4,4);
Labs(:,:,1) = zeros(4,4);

for j=1:7
    if j==4
        Lr = Lry;
    else
        Lr = Lrz;
    end
    % M(:,:,7) and M(:,:,8) represent the first two joints of the second
    % branch of the parallelogram. In order for the representation to be
    % correct it must be M(:,:,4) = M(:,:,8), W(:,:,4) = W(:,:,8) and 
    % H(:,:,4) = H(:,:,8)
    if j==6
        Labs(:,:,j) = M(:,:,1)*Lr/M(:,:,1);
        Wrel_0 = Labs(:,:,j)*Qd(j);
        Hrel_0 = Wrel_0^2+Labs(:,:,j)*Qdd(j);
        Habs(:,:,j+1) = Habs(:,:,1)+Hrel_0+2*Wabs(:,:,1)*Wrel_0;
        Wabs(:,:,j+1) = Wabs(:,:,1)+Wrel_0;
        M(:,:,j+1)=M(:,:,1)*M(:,:,j+1);
    else
        Labs(:,:,j) = M(:,:,j)*Lr/M(:,:,j);
        Wrel_0 = Labs(:,:,j)*Qd(j);
        Hrel_0 = Wrel_0^2+Labs(:,:,j)*Qdd(j);
        Habs(:,:,j+1) = Habs(:,:,j)+Hrel_0+2*Wabs(:,:,j)*Wrel_0;
        Wabs(:,:,j+1) = Wabs(:,:,j)+Wrel_0;
        M(:,:,j+1)=M(:,:,j)*M(:,:,j+1);
    end
end
Mabs = M;
end

