%
% test of motion law "tre tratti" (three steps) (see G.Legnani, "Robotica Indusyriale", pag 238)
%
clc
clear all
close all
%
T = 5 % motion time
n = 100 % n. of points of the law
dT = T/(n-1) % time step
xi = 5 % initial value
xf = 15 % final value
dx = xf-xi % total motion
l1 = 0.4 % percentage of motion time (1st step) 0.4=40%
l3 = 0.2 % percentage of motion time (2nd step) 0.2=20%

for i=1:n
    t=(i-1)*T/(n-1); % time from 0 to T with step dT
    tt(i)=t;
    [x(i),v(i),a(i)]=tretratti(t,T,xi,dx,l1,l3);
end

h1=figure(1)
set(h1,'name','Motion law - 3 steps')
plot(tt,x,  tt,v, tt, a,[tt(1) tt(end)], [0 0],'k') % plot law of motion
legend('s','v','a')
grid on

%start test of correctness

x_p=diff(x)/dT; % evaluate velocity by time differentiation of position
h2=figure(100)
set(h2,'name','check velocity')
plot(tt,v, tt(1:end-1),x_p,[tt(1) tt(end)], [0 0],'k') % compare true velocity and numerical estimation
grid on

x_pp=diff(v)/dT; % evaluate acceleration by time differentiation of velocity
h3=figure(101)
set(h3,'name','check acceleration')
plot(tt,a, tt(1:end-1),x_pp,[tt(1) tt(end)], [0 0],'k') % compare true acceleration and numerical estimation
grid on

