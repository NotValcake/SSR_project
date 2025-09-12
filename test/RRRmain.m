% Plot of trajectory and robot configuration(s)
h = 0.2;
l1 = 0.8;
l2 = 0.7;
l3 = 0.4;

L = [h l1 l2 l3]'; % robot dimensions

% Q… create trajectory in joint space
Qi = [-0.3 2.5 1]';
Qf = [0.3 1.5 0.15]';
cycloidal()
% x,y,z create trajectory in working space by direct kinematics


figure(1)
hold on
grid on
tit = title('Trajectory in the working space');
xlabel('x')
ylabel('y')
zlabel('z')
plot3(x,y,z,'g') % plot xyz trajectory
PlotRRR(Q(:,1),L,'r',1) % Robot in initial position
PlotRRR(Q(:,end),L,'b',1) % Robot in final position
legend('Robot in Si','Robot in Sf');
axis equal
