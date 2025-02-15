function [Q, Qd, Qdd] = invkin(L, P, Pd, Pdd)
% INVKIN Computes the joint positions, velocities, and accelerations for the given robot.
%
% [Q, Qd, Qdd] = invkin(L, P, Pd, Pdd) calculates the joint positions (Q),
% velocities (Qd), and accelerations (Qdd) for a planar robotic manipulator,
% given the link length vector (L), the desired end-effector position (P),
% velocity (Pd), and acceleration (Pdd).
%
% INPUT:
%   L   - Vector of link lengths [L1, L2, L3, L4, L5].
%   P   - Desired end-effector position [Px; Py; Pz].
%   Pd  - Desired end-effector velocity [Pxd; Pyd; Pzd].
%   Pdd - Desired end-effector acceleration [Pxdd; Pydd; Pzdd].
%
% OUTPUT:
%   Q   - Vector of joint positions [theta1; theta2; theta3].
%   Qd  - Vector of joint velocities [theta1d; theta2d; theta3d].
%   Qdd - Vector of joint accelerations [theta1dd; theta2dd; theta3dd].
%
% FUNCTION DESCRIPTION:
%   The invkin function computes the joint angles (Q), velocities (Qd), and
%   accelerations (Qdd) for a 3-degree-of-freedom robotic arm using inverse
%   kinematics. The Newton-Raphson method is used to determine the joint angles
%   based on the position constraints.
%
% ------------------------------------------------------------------------------

% Jacobian matrix
J = @(Q) [
    -L(2) * sin(Q(1)),                     0,                    -L(5) * cos(Q(3));
     L(2) * cos(Q(1)), -sin(Q(2)) * (L(4) + L(5) * cos(Q(3))),   -L(5) * cos(Q(2)) * sin(Q(3));
                    0, -cos(Q(2)) * (L(4) + L(5) * cos(Q(3))),    L(5) * sin(Q(2)) * sin(Q(3))
];

% Jacobian derivative
Jd = @(Q, Qd) [
    -L(2) * cos(Q(1)) * Qd(1),                                     0,                   L(5) * sin(Q(3)) * Qd(3);
    -L(2) * sin(Q(1)) * Qd(1), -cos(Q(2)) * (L(4) + L(5) * cos(Q(3))) * Qd(2) + L(5) * sin(Q(2)) * sin(Q(3)) * Qd(3),   L(5) * sin(Q(2)) * sin(Q(3)) * Qd(2) - L(5) * cos(Q(2)) * cos(Q(3)) * Qd(3);
                               0,  sin(Q(2)) * (L(4) + L(5) * cos(Q(3))) * Qd(2) + L(5) * cos(Q(2)) * sin(Q(3)) * Qd(3), L(5) * cos(Q(2)) * sin(Q(3)) * Qd(2) + L(5) * sin(Q(2)) * cos(Q(3)) * Qd(3)
];

% Position constraint equations
F = @(Q)[
    L(1) - L(1)/2 + L(2) * cos(Q(1)) - L(5) * sin(Q(3));
    L(3) + L(4) * cos(Q(2)) + L(2) * sin(Q(1)) + L(5) * cos(Q(2)) * cos(Q(3));
    -L(4) * sin(Q(2)) - L(5) * cos(Q(3)) * sin(Q(2))
];

% Initial estimation of joint angles based on desired position P
Q0 = [0, 0, 0]'; % Initial joint angle guess

% Solve the nonlinear system for Q using Newton-Raphson method
Q = newtonraphson(F, J, Q0, P, 1e-15, 1e3);

% Compute joint velocity Qd
Qd = J(Q) \ Pd;

% Compute joint acceleration Qdd
Qdd = J(Q) \ (Pdd - Jd(Q, Qd) * Qd);

end
