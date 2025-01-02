% Definire variabili simboliche
syms theta_2p theta_4 theta_5 real
syms l1p l2p l3p l3pp l4 l5 x y z a real

% Definire le equazioni del sistema
eq1 = x == l1p + l3p + l2p*cos(theta_2p) + l5*sin(theta_5);
eq2 = y == l3pp - l4*cos(theta_4) + l2p*sin(theta_2p) - l5*cos(theta_4)*cos(theta_5);
eq3 = z == l4*sin(theta_4) - l5*sin(theta_4)*cos(theta_5);

(l2p*sin(theta_2p))^2

(a - l4*cos(theta_4) - l5*cos(theta_4)*cos(theta_5))^2
