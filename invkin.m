function [Q, Qd, Qdd] = invkin(L, P, Pd, Pdd)
% INVKIN Calcola le posizioni, velocità e accelerazioni dei giunti il robot assegnato.
% 
% [Q, Qd, Qdd] = invkin(L, P, Pd, Pdd) calcola la posizione (Q),
% la velocità (Qd) e l'accelerazione (Qdd) dei giunti per il robot planare 
% assegnato, dato il vettore di lunghezze dei link (L), la posizione del gripper
% desiderata (P), la velocità (Pd) e l'accelerazione (Pdd).
%
% INPUT:
%   L   - Vettore di lunghezze dei link del robot (ad esempio, L = [L1, L2, L3, L4, L5]).
%   P   - Vettore di posizione desiderata del punto finale [Px; Py; Pz].
%   Pd  - Vettore di velocità desiderata del punto finale [Pxd; Pyd; Pzd].
%   Pdd - Vettore di accelerazione desiderata del punto finale [Pxdd; Pydd; Pzdd].
%
% OUTPUT:
%   Q   - Vettore di posizioni [theta1; theta2; theta3].
%   Qd  - Vettore di velocità dei giunti [theta1d; theta2d; theta3d].
%   Qdd - Vettore di accelerazioni dei giunti [theta1dd; theta2dd; theta3dd].
%
% DESCRIZIONE DELLA FUNZIONE:
%   La funzione invkin calcola gli angoli dei giunti (Q), le velocità (Qd) e
%   le accelerazioni (Qdd) per un braccio robotico a 3 gradi di libertà utilizzando 
%   il metodo della cinematica inversa. Si basa sull'uso del metodo di Newton-Raphson 
%   per determinare gli angoli articolari dati i vincoli di posizione.
%
% ------------------------------------------------------------------------------

% Matrice Jacobiana
J = @(Q) [
    -L(2) * sin(Q(1)),                     0,                    -L(5) * cos(Q(3));
     L(2) * cos(Q(1)), -sin(Q(2)) * (L(4) + L(5) * cos(Q(3))),   -L(5) * cos(Q(2)) * sin(Q(3));
                    0, -cos(Q(2)) * (L(4) + L(5) * cos(Q(3))),    L(5) * sin(Q(2)) * sin(Q(3))
];

% Derivata della matrice Jacobiana
Jd = @(Q, Qd) [
    -L(2) * cos(Q(1)) * Qd(1),                                     0,                   L(5) * sin(Q(3)) * Qd(3);
    -L(2) * sin(Q(1)) * Qd(1), -cos(Q(2)) * (L(4) + L(5) * cos(Q(3))) * Qd(2) + L(5) * sin(Q(2)) * sin(Q(3)) * Qd(3),   L(5) * sin(Q(2)) * sin(Q(3)) * Qd(2) - L(5) * cos(Q(2)) * cos(Q(3)) * Qd(3);
                               0,  sin(Q(2)) * (L(4) + L(5) * cos(Q(3))) * Qd(2) + L(5) * cos(Q(2)) * sin(Q(3)) * Qd(3), L(5) * cos(Q(2)) * sin(Q(3)) * Qd(2) + L(5) * sin(Q(2)) * cos(Q(3)) * Qd(3)
];

% Equazioni di vincolo di posizione
F = @(Q)[
    L(1) - L(1)/2 + L(2) * cos(Q(1)) - L(5) * sin(Q(3));
    L(3) + L(4) * cos(Q(2)) + L(2) * sin(Q(1)) + L(5) * cos(Q(2)) * cos(Q(3));
    -L(4) * sin(Q(2)) - L(5) * cos(Q(3)) * sin(Q(2))
    ];

% Stima iniziale per Q basata sulla posizione desiderata P
% theta2i = atan2(P(2), P(1))/2;   % Primo giunto: angolo della proiezione di P su x-y
% theta4 = atan2(P(3), P(1));      % Secondo giunto:  angolo della proiezione di P su z-y
% theta5 = atan2(P(2), P(1))/2;    % Terzo giunto: angolo della proiezione di P su x-y
Q0 = [0, 0, 0]'; % Inizializzazione degli angoli di partenza

% Risoluzione del sistema non lineare per Q usando Newton-Raphson
Q = newtonraphson(F, J, Q0, P, 1e-15, 1e3);

% Calcolo della velocità articolare Qd
Qd = J(Q) \ Pd;

% Calcolo dell'accelerazione articolare Qdd
Qdd = J(Q) \ (Pdd - Jd(Q, Qd) * Qd);

end
