%PROVASCARA 
% file di prova cinematica diretta ed inversa robot SCARA

clc
clear all
close all

l1=100, l2=70  %link length
L=[l1; l2]';
S=[100;-100]   %gripper position

figure(1)
% inverse kinematics
Q1=SCARAinv(S,L,1)  %1st solution (beta>0)
Q2=SCARAinv(S,L,-1) %2nd solution (beta<0)

PlotAreaSCARA(L,1) %plot working area
PlotSCARA(Q1,L,'r',1)   %plot scara 1 sol.
PlotSCARA(Q2,L,'b',1)   %plot scara 2 sol.
hold off

S1=SCARAdir(Q1,L) %test 1st solution (dir. kin.)
S2=SCARAdir(Q2,L) %test 2nd solution (dir. kin.)

% velocity and acceleration

Q=Q1 % choose solution n. 1
Qp = [ 1 0]' % joint velocities
Qpp = [1 1]' % joint accelerations

J=SCARAjac(Q,L)
Jp = SCARAjacP(Q,Qp,L)

Sp = J * Qp  % gripper velocity (direct kinematics)
Spp = J * Qpp + Jp * Qp  % gripper acceleration (dir. kinematics)

% inverse kinematics for debug

Qp1 = J^-1 * Sp
Qpp1 = J^-1 * (Spp-Jp*Qp)
