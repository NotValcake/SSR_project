
function plotellipsoid(cx, cy, cz, J)
% plotellipsoid Plotta l'ellissoide di manipolabilità 3D
% dato un Jacobiano J 3x3.
%
% INPUT:
%   J - Matrice Jacobiana 3x3 del manipolatore
%
% Esempio di utilizzo:
%   J = rand(3,3); % Jacobiano casuale 3x3
%   plot_manipulability_ellipsoid(J);

% Controlla che J sia una matrice 3x3
assert(all(size(J) == [3, 3]), 'Il Jacobiano deve essere una matrice 3x3');

% Calcola JJ^T per ottenere la matrice di covarianza dell'ellissoide
A = J' * J;

% Decomposizione ai valori singolari per ottenere l'orientamento e le lunghezze
[V, D] = eig(A);

% Lunghezze degli assi dell'ellissoide di manipolabilità
R = real(sqrt(diag(D)))';

[X, Y, Z] = ellipsoid(0,0,0,R(1),R(2),R(3));

% Appiattisci le matrici X, Y, Z in vettori e crea una matrice di punti
ellipsoid_points = [X(:)'; Y(:)'; Z(:)'];

% Applica la rotazione con la matrice V
rotated_points = V * ellipsoid_points;

% Ricostruisce le matrici per l'ellissoide ruotato
X = reshape(rotated_points(1, :), size(X)) + cx;
Y = reshape(rotated_points(2, :), size(Y)) + cy;
Z = reshape(rotated_points(3, :), size(Z)) + cz;

% Plotta l'ellissoide
surf(X, Y, Z, 'FaceAlpha', 0.5, 'EdgeColor','interp');
colormap("turbo");
axis equal;
grid on;
end

