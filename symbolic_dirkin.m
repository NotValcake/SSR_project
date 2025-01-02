% Definire le variabili simboliche
syms l1i l2i l3i l3ii l1ii l4 l5 ...
     theta2i theta3i theta1ii theta3ii ...
     theta4 theta5

% Matrice M_01'
M_01i = [1, 0, 0, l1i;
              0, 1, 0, 0;
              0, 0, 1, 0;
              0, 0, 0, 1];

% Matrice M_1'2'
M_1i2i = [cos(theta2i), -sin(theta2i), 0, l2i*cos(theta2i);
                    sin(theta2i),  cos(theta2i), 0, l2i*sin(theta2i);
                    0,                  0,                 1, 0;
                    0,                  0,                 0, 1];

% Matrice M_2'3
M_2i3 = [cos(theta3i - pi/2), -sin(theta3i - pi/2), 0, l3i*cos(theta3i) + l3ii*cos(theta3i - pi/2);
              sin(theta3i - pi/2),  cos(theta3i - pi/2), 0, l3i*sin(theta3i) + l3ii*sin(theta3i - pi/2);
              0,                        0,                          1, 0;
              0,                        0,                        0, 1];

% Matrice M_01''
M_01ii = [cos(theta1ii), -sin(theta1ii), 0, l1ii*cos(theta1ii);
                    sin(theta1ii),  cos(theta1ii), 0, l1ii*sin(theta1ii);
                    0,                       0,                        1, 0;
                    0,                       0,                        0, 1];

% Matrice M_1''3
M_1ii3 = [cos(theta3ii + pi/2), -sin(theta3ii + pi/2), 0, l3i*cos(theta3i) + l3ii*cos(theta3ii + pi/2);
                    sin(theta3ii + pi/2),  cos(theta3ii + pi/2), 0, l3i*sin(theta3i) + l3ii*sin(theta3ii + pi/2);
                    0,                             0,                             1, 0;
                    0,                             0,                             0, 1];

% Matrice M_34
M_34 = [ cos(theta4), 0, sin(theta4), l4*cos(theta4);
                   0, 1, 0,           0;
        -sin(theta4), 0, cos(theta4), -l4*sin(theta4);
                   0, 0, 0,           1];

% Matrice M_45
M_45 = [cos(theta5), -sin(theta5), 0, l5*cos(theta5);
        sin(theta5),  cos(theta5), 0, l5*sin(theta5);
        0,           0,           1, 0;
        0,           0,           0, 1];

M_03 = M_01i * M_1i2i * M_2i3;

M_03 = [0, -1, 0, l1i+l2i*cos(theta2i)-l3i;
        1, 0, 0, l2i*sin(theta2i)+l3ii;
        0, 0, 1, 0;
        0, 0, 0, 1];

M_05 = M_03*M_34*M_45
    
% M_05 =
% 
% [            -sin(theta5),             -cos(theta5),           0,                         l1i - l3i + l2i*cos(theta2i) - l5*sin(theta5)]
% [ cos(theta4)*cos(theta5), -cos(theta4)*sin(theta5), sin(theta4), l3ii + l4*cos(theta4) + l2i*sin(theta2i) + l5*cos(theta4)*cos(theta5)]
% [-cos(theta5)*sin(theta4),  sin(theta4)*sin(theta5), cos(theta4),                           -l4*sin(theta4) - l5*cos(theta5)*sin(theta4)]
% [                       0,                        0,           0,                                                                     1]
 
syms x y z

EQ(1) = x == M_05(1, end);
EQ(2) = y == M_05(2, end);
EQ(3) = z == M_05(3, end);

J = jacobian(M_05(1:3,end),[theta2i, theta4, theta5])

invJ = inv(J)

detJ = det(J)
%%
% Supponiamo di voler assegnare dei valori numerici a l2i, l4, e l5
l2i_val = 1.0;   % ad esempio 1.0
l4_val = 0.8    ;    % ad esempio 2.0
l5_val = 1.5;    % ad esempio 1.5

% Sostituisci le costanti in detJ
detJ_numeric = subs(detJ, [l2i, l4, l5], [l2i_val, l4_val, l5_val]);

% detJ_numeric =
% 
%   function_handle with value:
% 
%     @(theta4,theta5,theta2i)cos(theta4).*cos(theta5).^2.*cos(theta2i).*(9.0./4.0)+cos(theta4).^2.*sin(theta5).*sin(theta2i).*(6.0./5.0)+sin(theta4).^2.*sin(theta5).*sin(theta2i).*(6.0./5.0)+cos(theta4).*cos(theta5).*cos(theta2i).*(6.0./5.0)+cos(theta4).^2.*cos(theta5).*sin(theta5).*sin(theta2i).*(9.0./4.0)+cos(theta5).*sin(theta4).^2.*sin(theta5).*sin(theta2i).*(9.0./4.0)
% 

detJ_numeric = matlabFunction(detJ_numeric)

detJ_numeric(112, 0, pi/2)

detJ_numeric(112, acos(-l4_val/l5_val), 214)

detJ_numeric(pi/2, 0, 113)

detJ_numeric(113, pi/2, 0)
