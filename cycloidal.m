function [x,xp,xpp]=cycloidal(t,T,S0,dS)
%
% legge di moto TreTratti (acc.costante)
%
% t tempo per cui calcolare la legge
% T tempo di azionamento
% S0 posizione iniziale
% dS ampiezza movimento

% si assume Vini=Vfin=0
%
x = dS*(t/T-1/(2*pi)*sin(2*pi*t/T))+S0;
xp = dS/T*(1-cos(2*pi*t/T));
xpp= dS/(T^2)*2*pi*sin(2*pi*t/T);

end