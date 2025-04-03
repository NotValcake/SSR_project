clear; close all; clc;

% Define the lengths of each link
L = [1, 1, 1, 1, 1];

% Mass of each link
m = [1 1 1 1 1];

% Inertia tensor for each link, using the equivalent 3 mass approximation
% initialization of inertia matrix
I (:,:,1) = zeros(3,3);
G (:,:,1) = zeros(1,3);
J (:,:,1) = zeros(4,4);

Hg = [0, 0, 0, -9.81;  % Gravitational acceleration matrix
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];

% Start configuration to goal configuration Si → Sf
P1 = [0.5 4 0];

letterX = [-1.0000   -0.5000         0    0.5000    1.0000    1.5000...
            1.8000    2.0000    1.8000    1.5000    1.0000    0.7500...
            0.5000    0.2000         0    0.2000    0.5000    1.0000...
            1.5000    1.8000    1.9000    1.8000    1.5000];

letterY = [3.0000    3.0000    3.0000    3.0000    3.0000    3.0000...
           2.8000    2.5000    2.2000    2.0000    1.9000    1.9500...
           2.0000    2.2000    2.5000    2.8000    3.0000    3.0000...
           3.0000    3.1000    3.2000    3.4000    3.5000];

% Place the trajectory on an inclined plane
a = -.3; b = .2; d=0; 
Z = @(x,y) a*x + b*y +d;
letterZ = Z(letterX, letterY);

coords = [P1(1) letterX P1(1);
    P1(2) letterY P1(2);
    P1(3) letterZ P1(3)];

Amax = [0.05,0.05,0.05]';
Vmax = [0.1,.1,.1]';

d = height(Amax);
theta = zeros(d, length(coords));

tic
parfor j = 1:length(coords)
    [theta(:,j), ~, ~] = invkin(L,coords(:,j),zeros(3,1),zeros(3,1));
end
toc
%% ------------------------ THREE STEP SEGMENT ------------------------ %%
dT=1/100; % time step

% Calculate minimum actuation time for each motor
[t1(1), t3(1), T(1)] = minactime(theta(1,2)-theta(1,1),Amax(1),Amax(1),Vmax(1),"ts");
[t1(2), t3(2), T(2)] = minactime(theta(2,2)-theta(2,1),Amax(2),Amax(2),Vmax(2),"ts");  
[t1(3), t3(3), T(3)] = minactime(theta(3,2)-theta(3,1),Amax(3),Amax(3),Vmax(3),"ts");
ll1 = t1./T;
ll3 = t3./T;
tt = 0:dT:max(T);
n = length(tt);

% Initialize position, velocity and acceleration vectors
Q = zeros(d, n);
Qd = zeros(d, n);
Qdd = zeros(d, n);

% Calculate joint position, velocity and acceleration from P1 to P2
for i=1:length(tt)
    for j=1:d
        % Check if the actuation is terminated for each actuator
        if tt(i) <= T(j)
            [Q(j,i), Qd(j,i), Qdd(j,i)] = tretratti(tt(i),T(j),theta(j,1),theta(j,2)-theta(j,1),ll1(j),ll3(j));
        else
            Q(j,i) = Q(j,i-1);
            Qd(j,i) = 0;
            Qdd(j,i) = 0;
        end
    end
end

% Plot the three step trajectory of each actuator
for i=1:d
    figure(i)
    hold on
    plot(tt, Q(i,:), "-", LineWidth=1)
    plot(tt, Qd(i,:), "-", LineWidth=1)
    plot(tt, Qdd(i,:), "-", LineWidth=1)
    grid on
    title('Actuator ' + string(i))
    legend('Q','Qd','Qdd')
end

%% ---------------------- LINE PARABOLAS SEGMENT ---------------------- %%

% Compute the trajectory through the d shape in the joint space
[q_lp, qd_lp, qdd_lp, tt_lp, T_lp, t_lp] = lineparabolas2(theta(:,2:end-1), Amax, Vmax, dT);

% Compute the trajectory through the d shape in the work space
% [x_lp, xd_lp, xdd_lp, tt_lp, T_lp, t_lp] = lineparabolas2(coords(:,2:end-1), Amax, Vmax, dT);
% tic
% for j=1:length(xd_lp)
%     [q_lp(:,j), qd_lp(:,j), qdd_lp(:,j)] = invkin(L,x_lp(:,j),xd_lp(:,j),xdd_lp(:,j));
% end
% toc

% Plot the lineparabolas trajectory
for j = 1:d
    figure(10+j)
    subplot(3,1,1)
    hold on
    title('Position ' + string(j))
    plot(tt_lp, q_lp(j,:), '-', Color = "#0072BD", LineWidth=1)
    plot(T_lp, theta(j,2:end-1), 'k.')
    for i = 1:length(T_lp)
        if i == 1
            plot([T_lp(i)+t_lp(i), T_lp(i)+(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        elseif i == length(T)
            plot([T_lp(i)-t_lp(i), T_lp(i)-(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        else
            plot([T_lp(i)+t_lp(i)/2, T_lp(i)+(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
            plot([T_lp(i)-t_lp(i)/2, T_lp(i)+ -(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        end
    end
    ylim([min(q_lp(j,:)), max(q_lp(j,:))])

    grid on
    legend('Q', 'theta')

    subplot(3,1,2)
    hold on
    title('Velocity ' + string(j))
    plot(tt_lp, qd_lp(j,:), '-', Color = "#D95319", LineWidth=1)
    plot([tt_lp(1), tt_lp(end)], [Vmax(j), Vmax(j)], 'r--')
    plot([tt_lp(1), tt_lp(end)], -[Vmax(j), Vmax(j)], 'r--')
    grid on
    legend('Qd', 'Qd_{max}', 'Qd_{min}')
    for i = 1:length(T_lp)
        if i == 1
            plot([T_lp(i)+t_lp(i), T_lp(i)+(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        elseif i == length(T)
            plot([T_lp(i)-t_lp(i), T_lp(i)-(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        else
            plot([T_lp(i)+t_lp(i)/2, T_lp(i)+(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
            plot([T_lp(i)-t_lp(i)/2, T_lp(i)+ -(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        end
    end
    ylim([-Vmax(j), Vmax(j)])

    subplot(3,1,3)
    hold on
    title('Acceleration ' + string(j))
    plot(tt_lp, qdd_lp(j,:), '-', Color = "#EDB120", LineWidth=1)
    plot([tt_lp(1), tt_lp(end)], [Amax(j), Amax(j)], 'r--')
    plot([tt_lp(1), tt_lp(end)], -[Amax(j), Amax(j)], 'r--')
    grid on
    legend('Qdd', 'Qdd_{max}', 'Qdd_{min}')
    for i = 1:length(T_lp)
        if i == 1
            plot([T_lp(i)+t_lp(i), T_lp(i)+(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        elseif i == length(T)
            plot([T_lp(i)-t_lp(i), T_lp(i)-(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        else
            plot([T_lp(i)+t_lp(i)/2, T_lp(i)+(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
            plot([T_lp(i)-t_lp(i)/2, T_lp(i)+ -(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        end
    end
    ylim([-Amax(j), Amax(j)])
end

%% ------------------------ CYCLOIDAL SEGMENT ------------------------ %%

% Calculate minimum actuation time for each motor
[t1(1), t3(1), T_c(1)] = minactime(theta(1,end)-theta(1,end-1),Amax(1),Amax(1),Vmax(1),"c");
[t1(2), t3(2), T_c(2)] = minactime(theta(2,end)-theta(2,end-1),Amax(2),Amax(2),Vmax(2),"c");  
[t1(3), t3(3), T_c(3)] = minactime(theta(3,end)-theta(3,end-1),Amax(3),Amax(3),Vmax(3),"c");
ll1 = t1./T;
ll3 = t3./T;
tt_c = 0:dT:max(T_c);

% Initialize position, velocity and acceleration vectors
q_c = zeros(d, length(tt_c));
qd_c = zeros(d, length(tt_c));
qdd_c = zeros(d, length(tt_c));

% Calculate joint position, velocity and acceleration from P1 to P2
for i=1:length(tt_c)
    for j=1:d
        % Check if the actuation is terminated for each actuator
        if tt_c(i) <= T_c(j)
            [q_c(j,i), qd_c(j,i), qdd_c(j,i)] = cycloidal(tt_c(i),T_c(j),theta(j,end-1),theta(j,end)-theta(j,end-1));
        else
            q_c(j,i) = q_c(j,i-1);
            qd_c(j,i) = 0;
            qdd_c(j,i) = 0;
        end
    end
end

% Plot the three step trajectory of each actuator
for i=1:d
    figure(20+i)
    hold on
    plot(tt_c, q_c(i,:), "-", LineWidth=1)
    plot(tt_c, qd_c(i,:), "-", LineWidth=1)
    plot(tt_c, qdd_c(i,:), "-", LineWidth=1)
    grid on
    title('Actuator ' + string(i))
    legend('Q','Qd','Qdd')
end

%% ------------------------ MERGE TRAJECTORIES ------------------------ %%

tt = [tt tt_lp+tt(end) tt_c+tt_lp(end)+tt(end)];
Q = [Q q_lp q_c];
Qd = [Qd qd_lp qd_c];
Qdd = [Qdd qdd_lp qdd_c];

% tt = tt_lp;
% Q = q_lp;
% Qd = qd_lp;
% Qdd = qdd_lp;

% Plot the total trajectory of each actuator
for i=1:d
    figure(30+i)
    hold on
    plot(tt, Q(i,:), "-", LineWidth=1)
    plot(tt, Qd(i,:), "-", LineWidth=1)
    plot(tt, Qdd(i,:), "-", LineWidth=1)
    grid on
    title('Actuator ' + string(i))
    legend('Q','Qd','Qdd')
end

Phie5 = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];

% Initialize position, velocity and acceleration vectors
P = zeros(4,4,length(tt));
Pd = zeros(4,4,length(tt));
Pdd = zeros(4,4,length(tt));

for t=1:length(tt) % generate motion in working space
    
    % Solve forward kinematic
    [M, W, H, Labs] = dirkin(L, Q(:,t), Qd(:,t), Qdd(:,t));

    % Compute position, velocity and acceleration of end effector
    P(:,:,t) = M(:,:,6);
    Pd(:,:,t) = (W(:,:,6)*M(:,:,6));
    Pdd(:,:,t) = (H(:,:,6)*M(:,:,6));

    % Inertia moments calculations
    % for j = 1:1:length(L)
    %     % calculate inertia moments with respect to the center of gravity
    %     if j==3 % link 3 is a special case
    % 
    %         % link l3'
    %         a = L(1)/2;
    %         b = L(1)/2;
    %         Jg = 1/12*m(1)*(a+b)^2;
    %         I3ig = [Jg 0 0; 0 0 0; 0 0 Jg];
    %         M33i = [1 0 0 -L(3)
    %             0 1 0 0
    %             0 0 1 0
    %             0 0 0 1];
    %         J3ig = pseudoinertia(I3ig, m(1), [0 0 0]);
    % 
    % 
    %         % link l3''
    %         a = L(3)/2;
    %         b = L(3)/2;
    %         Jg = 1/12*m(3)*(a+b)^2;
    %         I3iig = [0 0 0; 0 Jg 0; 0 0 Jg];
    %         M33ii = [1 0 0 -L(3)/2
    %             0 1 0 0
    %             0 0 1 0
    %             0 0 0 1];
    %         J3iig = pseudoinertia(I3iig, m(3), [0 0 0]);
    %         J(:,:,3) = J3ig + J3iig;
    %         J(:,:,3) = M(:,:,4)*J(:,:,3)*M(:,:,4)';
    %     else
    %         a = L(j)/2;
    %         b = L(j)/2;
    %         m1 = m(j)/6;
    %         m2 = m(j)/6;
    %         mg = 2/3*m(j);
    %         Jg = 1/12*m(j)*(a+b)^2;
    % 
    %         I(:,:,j) = [0 0 0; 0 Jg 0; 0 0 Jg];
    %         G(:,:,j) = [-b 0 0];
    %         J(:,:,j) = pseudoinertia(I,m(j),G(:,:,j));
    %         J(:,:,j) = M(:,:,j+1)*J(:,:,j)*M(:,:,j+1);
    %         if j==2
    %             J1ii = M(:,:,7)*J(:,:,j)*M(:,:,7);
    %         end
    %     end
    % end

    % Phie0 = M(:,:,6)*Phie5*M(:,:,6)';
    % [Phi, phi(i,:)] = invdyn(M,H,Labs,J,Phie0,L);
    % 
    % % Kinetic and potential energy computation
    % t(i) = 0;
    % u (i) = 0;
    % for l= 1:1:size(J,3)
    %     if l == 2
    %         t(i) = t(i) + 1/2*trace(W(:,:,7)*J(:,:,l)*W(:,:,7)');
    %     elseif l == 1
    %         t(i) = t(i) + 1/2*trace(W(:,:,l+2)*J(:,:,l)*W(:,:,l+2)');
    %     else
    %         t(i) = t(i) + 1/2*trace(W(:,:,l+1)*J(:,:,l)*W(:,:,l+1)');
    %     end
    %     u(i) = u(i) -trace(Hg*J(:,:,l));
    % end
    % p(i) = pseudot(Phi(:,:,1),W(:,:,3)) + pseudot(Phi(:,:,2),W(:,:,4)) + pseudot(Phi(:,:,4),W(:,:,5)) + pseudot(Phi(:,:,5),W(:,:,6));
end

%% --------------------------- VISUALIZATION --------------------------- %%
% Plot start and end configuration
plotmanipulator(Q(:,1), L, 'r', 100);
for i=2:100:length(tt)
    plotmanipulator(Q(:,i), L, 'g', 100);
end
plotmanipulator(Q(:,end), L, 'b', 100);

figure(102)
hold on
% Plot the computed position velocity and acceleration
plot(tt, reshape(P(1,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pd(1,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pdd(1,end,:),1,[]), "-", LineWidth=1)
% Plot numeric velocity and acceleration
plot(tt(2:end), diff(reshape(P(1,end,:),1,[]))/dT, "-.", LineWidth=1)
plot(tt(2:end),diff(reshape(Pd(1,end,:),1,[]))/dT, "-.", LineWidth=1)
grid on
title('X Linear')
legend('x','xd','xdd','xdn','xddn')

figure(103)
hold on
plot(tt, reshape(P(2,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pd(2,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pdd(2,end,:),1,[]), "-", LineWidth=1)
% Plot numeric velocity and acceleration
plot(tt(2:end), diff(reshape(P(2,end,:),1,[]))/dT, "-.", LineWidth=1)
plot(tt(2:end),diff(reshape(Pd(2,end,:),1,[]))/dT, "-.", LineWidth=1)
grid on
title('Y Linear')
legend('y','yd','ydd','ydn','yddn')

figure(104)
hold on
plot(tt, reshape(P(3,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pd(3,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pdd(3,end,:),1,[]), "-", LineWidth=1)
% Plot numeric velocity and acceleration
plot(tt(2:end), diff(reshape(P(3,end,:),1,[]))/dT, "-.", LineWidth=1)
plot(tt(2:end),diff(reshape(Pd(3,end,:),1,[]))/dT, "-.", LineWidth=1)
grid on
title('Z Linear')
legend('z','zd','zdd','zdn','zddn')

figure(105)
hold on
plot3(reshape(P(1,end,:),1,[]), reshape(P(2,end,:),1,[]), reshape(P(3,end,:),1,[]), "-", LineWidth=1)
plot3(coords(1,:), coords(2,:), coords(3,:), "*-.")
% plot([0 T],[0 0], 'k')
grid on
title('Working space trajectory')

% figure(5)
% hold on
% plot(tt, phi(:,1))
% plot(tt, phi(:,2))
% plot(tt, phi(:,3))
% % plot([0 T],[0 0], 'k')
% grid on
% title('Motor torques')
% legend('theta2''','theta4','theta5')
% 
% figure(6)
% hold on
% plot(tt, t)
% plot(tt, u)
% grid on
% title('Energy')
% legend('Ek', 'Ep')
% 
% de = diff(t+u)/dT;
% 
% figure(7)
% hold on
% plot(tt(2:end), de)
% plot(tt, p)
% grid on
% title('Power')
% legend('d(T+U)/dt', 'W')
% 
% figure(8)
% hold on
% plot(tt, Q(1,:), "-", LineWidth=1.5)
% plot(tt, Qd(1,:), "-", LineWidth=1.5)
% plot(tt, Qdd(1,:), "-", LineWidth=1.5)
% % plot([0 T],[0 0], 'k')
% grid on
% title('x Linear')
% legend('x','xd','xdd')
% 
% figure(9)
% hold on
% plot(tt, Q(2,:), "-", LineWidth=1.5)
% plot(tt, Qd(2,:), "-", LineWidth=1.5)
% plot(tt, Qdd(2,:), "-", LineWidth=1.5)
% % plot([0 T],[0 0], 'k')
% grid on
% title('Y Linear')
% legend('y','yd','ydd')
% 
% figure(10)
% hold on
% plot(tt, Q(3,:), "-", LineWidth=1.5)
% plot(tt, Qd(3,:), "-", LineWidth=1.5)
% plot(tt, Qdd(3,:), "-", LineWidth=1.5)
% % plot([0 T],[0 0], 'k')
% grid on
% title('Z Linear')
% legend('z','zd','zdd')
