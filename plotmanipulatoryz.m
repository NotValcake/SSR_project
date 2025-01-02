function [x5, y5, z5] = plotmanipulator(Q,L,colore,fig)
% PLOTMANIPULATOR Questa funzione plotta il manipolatore assegnato, date le
% posizioni degli attuatori Q e le dimensioni dei link L
%
% Parametri di input:
% - Q: Vettore degli angoli degli attuatori, specificati in radianti.
%      Q è un vettore di 3 elementi dove:
%         Q(1) = theta2i  -> angolo per il giunto J2' (posizione del giunto 2')
%         Q(2) = theta4   -> angolo per il giunto J4 (posizione del giunto 3)
%         Q(3) = theta5   -> angolo per il giunto J5 (posizione del giunto 4)
% - L: Vettore delle lunghezze dei link. Deve essere di 5 elementi:
%         L(1) = l1i      -> lunghezza del link l1'
%         L(2) = l2i      -> lunghezza del link l2'
%         L(3) = l3ii     -> lunghezza del link l3''
%         L(4) = l4       -> lunghezza del link l4
%         L(5) = l5       -> lunghezza del link l5
% - colore: Specifica il colore dei link e dei giunti del manipolatore
%           nel plot. Ad esempio, 'r' per rosso, 'b' per blu, ecc.
% - fig: Numero della figura in cui viene plottato il manipolatore.
%
% Outputs:
%   x5   - coordinata x del gripper
%   y5   - coordinata y del gripper
%   z5   - coordinata z del gripper
%
% Descrizione:
% 1. La funzione calcola le coordinate (x, y, z) di ciascun giunto del
%    manipolatore robotico in base agli angoli e alle lunghezze specificate.
% 2. Vengono definiti gli angoli relativi per i vari giunti e i punti 
%    intermedi in modo da rispettare la configurazione geometrica del
%    manipolatore.
% 3. Vengono plottati i segmenti tra i giunti (link) per mostrare il
%    manipolatore completo in 3D.
%
% Esempio di utilizzo:
%   Q = [pi/4, pi/6, pi/3];
%   L = [1.0, 0.8, 0.5, 1.2, 0.7];
%   colore = 'b';
%   fig = 1;
%   plotmanipulator(Q, L, colore, fig);
%
% Nota:
% - Viene mantenuta la proporzione degli assi con `axis equal` per una
%   visualizzazione corretta.
%

figure(fig) % fig figure number
l1i  = L(1);
l2i  = L(2);
l3ii = L(3);
l1ii = l2i;
l3i  = l1i/2;
l4   = L(4);
l5   = L(5);

theta2i  = Q(1);
theta3i  = pi-theta2i;
theta1ii = theta2i;
theta4   = Q(2);
theta5   = Q(3);

y0 = 0;
z0 = 0;

y1i = y0;
z1i = z0;

y1ii = l1ii*sin(theta1ii);
z1ii = z0;

y2i = y1i + l2i*sin(theta2i);
z2i = z1i;

y3 = y1ii + l3ii;
z3 = z1ii;

y3i = (y2i + y1ii)/2;
z3i = (z2i + z1ii)/2;

y4 = y3 + l4*cos(theta4);
z4 = z3 - l4*sin(theta4);

y5 = y4 + l5*cos(theta4)*cos(theta5);
z5 = z4 - l5*sin(theta4)*cos(theta5);

hold on

% manipulator
plot([y1i y2i y3i y3 y4 y5],[z1i z2i z3i z3 z4 z5], 'LineWidth',2,'color',colore);
plot([y0 y1ii y3i],[z0 z1ii z3i], 'LineWidth', 2,'color', colore);
% base
plot([y0 y1i],[z0 z1i],'*-','color','k', 'LineWidth', 2, 'MarkerSize',5);
% joints
plot([y0 y1i y1ii y2i y3 y4 y5],[z0 z1i z1ii z2i z3 z4 z5], 'o','color', colore, 'MarkerSize', 5, 'MarkerFaceColor',colore);
% end effector
plot(y5,z5, 'diamond','color', colore, 'MarkerSize', 10, 'MarkerFaceColor', colore);
xlabel y
ylabel z
axis equal
grid on