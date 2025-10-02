clear; close all; clc;
addpath("simulation\");

% Define the lengths of each link
L = [1, 1.1, 1, 1, .9];

numlinks = 5;

% Mass of each link (Assuming robot is made of Aluminium
m = [3.245734 3.461734 5.948601 3.245734 3.029734];

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

letterX = [-0.8000   -0.5000         0    0.5000    1.0000    1.5000...
            1.7000    1.9000    1.7000    1.5000    1.0000    ...
            0.5000    0.3000    0.1000    0.3000    0.5000    1.0000...
            1.5000    1.7000    1.8000    1.7000    1.5000];

letterY = [3.2000    3.2000    3.2000    3.2000    3.2000    3.2000...
           3.0000    2.7000    2.4000    2.2000    2.1000    2.2000...
           2.4000    2.7000    3.0000    3.2000    3.2000    3.2000...
           3.2500    3.4000    3.5500    3.6000];

% Place the trajectory on an inclined plane
a = -.3; b = .2; d=0; 
Z = @(x,y) a*x + b*y +d;
letterZ = Z(letterX, letterY);

coords = [P1(1) letterX P1(1);
    P1(2) letterY P1(2);
    P1(3) letterZ P1(3)];

Amax = [1,1,1]';
Vmax = [1.5,1.5,2]';

d = height(Amax);
theta = zeros(d, length(coords));

tic
[theta(:,1), ~, ~] = invkin(L,coords(:,1),zeros(3,1),zeros(3,1),[0,0,0]');
for j = 2:length(coords)
    [theta(:,j), ~, ~] = invkin(L,coords(:,j),zeros(3,1),zeros(3,1),theta(:,j-1));
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

% % Plot the three step trajectory of each actuator
% for i=1:d
%     figure(i)
%     hold on
%     plot(tt, Q(i,:), "-", LineWidth=1)
%     plot(tt, Qd(i,:), "-", LineWidth=1)
%     p lot(tt, Qdd(i,:), "-", LineWidth=1)
%     grid on
%     title('Actuator ' + string(i))
%     legend('Q','Qd','Qdd')
%     xlabel("t [s]")
%     ylabel("Q [rad]    Qd [rad/s]    Qdd [rad/s^2]")
% end

%% ---------------------- LINE PARABOLAS SEGMENT ---------------------- %%

% Compute the trajectory through the d shape in the joint space
% [q_lp, qd_lp, qdd_lp, tt_lp, T_lp, t_lp] = lineparabolas(theta(:,2:end-1), Amax, Vmax, dT);

% Compute the trajectory through the d shape in the work space
[x_lp, xd_lp, xdd_lp, tt_lp, T_lp, t_lp] = lineparabolas(coords(:,2:end-1), Amax, Vmax, dT);
tic
[q_lp(:,1), qd_lp(:,1), qdd_lp(:,1)] = invkin(L,x_lp(:,1),xd_lp(:,1),xdd_lp(:,1),[0,0,0]');
for j=2:length(xd_lp)
    [q_lp(:,j), qd_lp(:,j), qdd_lp(:,j)] = invkin(L,x_lp(:,j),xd_lp(:,j),xdd_lp(:,j),q_lp(:,j-1));
end
toc

% % Plot the lineparabolas trajectory (works with joint space generated
% % trajectories)
% for j = 1:d
%     figure(10+j)
%     subplot(3,1,1)
%     hold on
%     title('Position ' + string(j))
%     plot(tt_lp, q_lp(j,:), '-', Color = "#0072BD", LineWidth=1)
%     plot(T_lp, theta(j,2:end-1), 'k.--')
%     xlabel("t [s]")
%     ylabel("Q [rad]")
%     for i = 1:length(T_lp)
%         if i == 1
%             plot([T_lp(i)+t_lp(i), T_lp(i)+(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%         elseif i == length(T)
%             plot([T_lp(i)-t_lp(i), T_lp(i)-(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%         else
%             plot([T_lp(i)+t_lp(i)/2, T_lp(i)+(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%             plot([T_lp(i)-t_lp(i)/2, T_lp(i)+ -(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%         end
%     end
%     ylim([min(theta(j,2:end-1)), max(theta(j,2:end-1))])
% 
%     grid on
%     legend('Q', 'theta')
% 
%     subplot(3,1,2)
%     hold on
%     title('Velocity ' + string(j))
%     plot(tt_lp, qd_lp(j,:), '-', Color = "#D95319", LineWidth=1)
%     plot([tt_lp(1), tt_lp(end)], [Vmax(j), Vmax(j)], 'r--')
%     plot([tt_lp(1), tt_lp(end)], -[Vmax(j), Vmax(j)], 'r--')
%     grid on
%     legend('Qd', 'Qd_{max}', 'Qd_{min}')
%     xlabel("t [s]")
%     ylabel("Qd [rad/s]")
%     for i = 1:length(T_lp)
%         if i == 1
%             plot([T_lp(i)+t_lp(i), T_lp(i)+(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%         elseif i == length(T)
%             plot([T_lp(i)-t_lp(i), T_lp(i)-(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%         else
%             plot([T_lp(i)+t_lp(i)/2, T_lp(i)+(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%             plot([T_lp(i)-t_lp(i)/2, T_lp(i)+ -(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%         end
%     end
%     ylim([-Vmax(j), Vmax(j)])
% 
%     subplot(3,1,3)
%     hold on
%     title('Acceleration ' + string(j))
%     plot(tt_lp, qdd_lp(j,:), '-', Color = "#EDB120", LineWidth=1)
%     plot([tt_lp(1), tt_lp(end)], [Amax(j), Amax(j)], 'r--')
%     plot([tt_lp(1), tt_lp(end)], -[Amax(j), Amax(j)], 'r--')
%     grid on
%     legend('Qdd', 'Qdd_{max}', 'Qdd_{min}')
%     xlabel("t [s]")
%     ylabel("Qdd [rad/s^2]")
%     for i = 1:length(T_lp)
%         if i == 1
%             plot([T_lp(i)+t_lp(i), T_lp(i)+(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%         elseif i == length(T)
%             plot([T_lp(i)-t_lp(i), T_lp(i)-(t_lp(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%         else
%             plot([T_lp(i)+t_lp(i)/2, T_lp(i)+(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%             plot([T_lp(i)-t_lp(i)/2, T_lp(i)+ -(t_lp(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
%         end
%     end
%     ylim([-Amax(j), Amax(j)])
% end

%% ------------------------ CYCLOIDAL SEGMENT ------------------------ %%

% Calculate minimum actuation time for each motor
[t1(1), t3(1), T_c(1)] = minactime(theta(1,end)-q_lp(1,end),Amax(1),Amax(1),Vmax(1),"c");
[t1(2), t3(2), T_c(2)] = minactime(theta(2,end)-q_lp(1,end),Amax(2),Amax(2),Vmax(2),"c");  
[t1(3), t3(3), T_c(3)] = minactime(theta(3,end)-q_lp(1,end),Amax(3),Amax(3),Vmax(3),"c");
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
            [q_c(j,i), qd_c(j,i), qdd_c(j,i)] = cycloidal(tt_c(i),T_c(j),q_lp(j,end),theta(j,end)-q_lp(j,end));
        else
            q_c(j,i) = q_c(j,i-1);
            qd_c(j,i) = 0;
            qdd_c(j,i) = 0;
        end
    end
end

% % Plot the three step trajectory of each actuator
% for i=1:d
%     figure(20+i)
%     hold on
%     plot(tt_c, q_c(i,:), "-", LineWidth=1)
%     plot(tt_c, qd_c(i,:), "-", LineWidth=1)
%     plot(tt_c, qdd_c(i,:), "-", LineWidth=1)
%     grid on
%     title('Actuator ' + string(i))
%     legend('Q','Qd','Qdd')
%     xlabel("t [s]")
%     ylabel("Q [rad]    Qd [rad/s]    Qdd [rad/s^2]")
% end

%% ------------------------ MERGE TRAJECTORIES ------------------------ %%

tt = [tt tt_lp+tt(end) tt_c+tt_lp(end)+tt(end)];
Q = [Q q_lp q_c];
Qd = [Qd qd_lp qd_c];
Qdd = [Qdd qdd_lp qdd_c];

% tt = tt_lp;
% Q = q_lp;
% Qd = qd_lp;
% Qdd = qdd_lp;

% % Plot the total trajectory of each actuator
% for i=1:d
%     figure(30+i)
%     hold on
%     plot(tt, Q(i,:), "-", LineWidth=1)
%     plot(tt, Qd(i,:), "-", LineWidth=1)
%     plot(tt, Qdd(i,:), "-", LineWidth=1)
%     grid on
%     title('Actuator ' + string(i))
%     legend('Q','Qd','Qdd')
%     xlabel("t [s]")
%     ylabel("Q [rad]    Qd [rad/s]    Qdd [rad/s^2]")
% end

%% ------------------ KNEMATIC AND DYNAMICS ANALYSIS ------------------ %%

% External forces acting on the end effector
Phie5 = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];

% Initialize position, velocity and acceleration vectors
P = zeros(4,4,length(tt));
Pd = zeros(4,4,length(tt));
Pdd = zeros(4,4,length(tt));

% Initialize torques vector
torques = zeros(length(tt),d);

for t=1:length(tt) % generate motion in working space
    
    % Solve forward kinematic
    [Mabs, Wabs, Habs, Labs, Wrel, Hrel] = dirkin(L, Q(:,t), Qd(:,t), Qdd(:,t));

    % Compute position, velocity and acceleration of end effector
    P(:,:,t) = Mabs(:,:,6);
    Pd(:,:,t) = (Wabs(:,:,6)*Mabs(:,:,6));
    Pdd(:,:,t) = (Habs(:,:,6)*Mabs(:,:,6));

    % Inertia moments calculations
    for j = 1:1:length(L)
        % calculate inertia moments with respect to the center of gravity
        if j==3 % link 3 is a special case

            % link l3'
            a = L(1)/2;
            b = L(1)/2;
            Jg = 1/12*m(1)*(a+b)^2;
            I(:,:,j) = [Jg 0 0; 0 0 0; 0 0 Jg];
            M33i = [1 0 0 -L(3)
                0 1 0 0
                0 0 1 0
                0 0 0 1];
            J3ig = pseudoinertia(I(:,:,j), m(1), [0 0 0]);


            % link l3''
            a = L(3)/2;
            b = L(3)/2;
            Jg = 1/12*m(3)*(a+b)^2;
            I3iig = [0 0 0; 0 Jg 0; 0 0 Jg];
            M33ii = [1 0 0 -L(3)/2
                0 1 0 0
                0 0 1 0
                0 0 0 1];
            J3iig = pseudoinertia(I3iig, m(3), [0 0 0]);
            J(:,:,3) = J3ig + J3iig;
            J(:,:,3) = Mabs(:,:,4)*J(:,:,3)*Mabs(:,:,4)';
        else
            a = L(j)/2;
            b = L(j)/2;
            m1 = m(j)/6;
            m2 = m(j)/6;
            mg = 2/3*m(j);
            Jg = 1/12*m(j)*(a+b)^2;

            I(:,:,j) = [0 0 0; 0 Jg 0; 0 0 Jg];
            G(:,:,j) = [-b 0 0];
            J(:,:,j) = pseudoinertia(I(:,:,j),m(j),G(:,:,j));
            J(:,:,j) = Mabs(:,:,j+1)*J(:,:,j)*Mabs(:,:,j+1)';
            if j==2
                J1ii = Mabs(:,:,7)*J(:,:,j)*Mabs(:,:,7)';
            end
        end
    end
    
    Phie0 = Mabs(:,:,6)*Phie5*Mabs(:,:,6)';
    [~, torques(t,:)] = invdyn(Habs,Labs,J,Phie0,Hg,L,Q(:,t),Qd(:,t),Qdd(:,t),5);
end
%% ---------------------------- SIMULATION ----------------------------- %%
% Set simulation parameters
W = 0.04; % Link width in m
T = 0.02; % Link thickness in m
rho = 2700; % Link density in kg/m^3

% Colors
red = [0.8 0.3 0]; yellow = [0.8 0.8 0]; blue = [0.0 0.5 0.8];
orange = [0.8 0.5 0.1]; black = [0 0 0]; alpha = 1;

% Set simulation inputs
Q1sim.time = tt'; Q2sim.time = tt'; Q3sim.time = tt';
Q1sim.signals.values = Q(1,:)'; 
Q2sim.signals.values = Q(2,:)'; 
Q3sim.signals.values = Q(3,:)';

% Extract points to visualize path in simulation
xpath = reshape(P(1,end,:),1,[]);
ypath = reshape(P(2,end,:),1,[]);
zpath = reshape(P(3,end,:),1,[]);

simout = sim("simulation\manipulator_sim_PID.slx",...
    'StartTime', num2str(tt(1)), ...
    'StopTime',  num2str(tt(end)));

%% --------------------------- VISUALIZATION --------------------------- %%
% Plot start and end configuration
plotmanipulator(Q(:,1), L, 'r', 100);
plotmanipulator(theta(:,2), L, 'k', 100);
plotmanipulator(theta(:,end-1), L, 'k', 100);

% Plot intermediate positions for debug
% for i=2:2000:length(tt)
%     plotmanipulator2(Q(:,i), L, 'g', 100);
% end
plotmanipulator(Q(:,end), L, 'b', 100);
plot3(reshape(P(1,end,:),1,[]), reshape(P(2,end,:),1,[]), reshape(P(3,end,:),1,[]), "-", LineWidth=1)
plot3(coords(1,:),coords(2,:),  coords(3,:), "*-.")

% plot3([-10,10],[0,0],[0,0],'k')
% plot3([0,0],[-10,10],[0,0],'k')
% plot3([0,0],[0,0],[-10,10],'k')

% limits to be used with plotmanipulator2 results
% xlim([-2 3])
% ylim([0 4])
% zlim([-3 3])
plot3([-2,3],[0,0],[0,0],'k')
plot3([0,0],[0,4],[0,0],'k')
plot3([0,0],[0,0],[-1,1],'k')
axis equal

title("Manipulator in workspace")
xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

figure(102)
hold on
% Plot the computed position velocity and acceleration
plot(tt, reshape(P(1,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pd(1,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pdd(1,end,:),1,[]), "-", LineWidth=1)
grid on
title('X Linear')
legend('x','xd','xdd')
xlabel("t [s]")
ylabel("x [m]    xd [m/s]    xdd [m/s^2]")

figure(103)
hold on
plot(tt, reshape(P(2,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pd(2,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pdd(2,end,:),1,[]), "-", LineWidth=1)
grid on
title('Y Linear')
legend('y','yd','ydd')
xlabel("t [s]")
ylabel("y [m]    yd [m/s]    ydd [m/s^2]")

figure(104)
hold on
plot(tt, reshape(P(3,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pd(3,end,:),1,[]), "-", LineWidth=1)
plot(tt, reshape(Pdd(3,end,:),1,[]), "-", LineWidth=1)
grid on
title('Z Linear')
legend('z','zd','zdd')
xlabel("t [s]")
ylabel("z [m]    zd [m/s]    zdd [m/s^2]")

figure(105)
hold on
plot3(reshape(P(1,end,:),1,[]), reshape(P(2,end,:),1,[]), reshape(P(3,end,:),1,[]), "-", LineWidth=1)
plot3(coords(1,:), coords(2,:), coords(3,:), "*-.")
% plot([0 T],[0 0], 'k')
grid on
title('Working space trajectory')
xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")
legend("Robot trajectory", "Objective trajectory")
view(45,45)

figure(200)
subplot(3,1,1)
hold on
grid on
title("Simulated vs. teorical x")
xlabel("t [s]")
ylabel("position [m]")
plot(simout.p.Time,simout.p.Data(:,1),LineWidth=1,DisplayName="x simulation")
plot(tt,reshape(P(1,end,:),1,[]),'--',LineWidth=1,DisplayName="x")
legend

subplot(3,1,2)
hold on
grid on
title("Simulated vs. teorical y")
xlabel("t [s]")
ylabel("position [m]")
plot(simout.p.Time,simout.p.Data(:,2),LineWidth=1,DisplayName="y simulation")
plot(tt,reshape(P(2,end,:),1,[]),'--',LineWidth=1,DisplayName="y")
legend

subplot(3,1,3)
hold on
grid on
title("Simulated vs. teorical z")
xlabel("t [s]")
ylabel("position [m]")
plot(simout.p.Time,simout.p.Data(:,3),LineWidth=1,DisplayName="z simulation")
plot(tt,reshape(P(3,end,:),1,[]),'--',LineWidth=1,DisplayName="z")
legend

figure(201)
subplot(3,1,1)
hold on
grid on
title("Simulated vs. teorical xd")
xlabel("t [s]")
ylabel("velocity [m/s]")
plot(simout.pd.Time,simout.pd.Data(:,1),LineWidth=1,DisplayName="xd simulation")
plot(tt,reshape(Pd(1,end,:),1,[]),'--',LineWidth=1,DisplayName="xd")
legend

subplot(3,1,2)
hold on
grid on
title("Simulated vs. teorical xd")
xlabel("t [s]")
ylabel("velocity [m/s]")
plot(simout.pd.Time,simout.pd.Data(:,2),LineWidth=1,DisplayName="yd simulation")
plot(tt,reshape(Pd(2,end,:),1,[]),'--',LineWidth=1,DisplayName="yd")
legend

subplot(3,1,3)
hold on
grid on
title("Simulated vs. teorical zd")
xlabel("t [s]")
ylabel("velocity [m/s]")
plot(simout.pd.Time,simout.pd.Data(:,3),LineWidth=1,DisplayName="zd simulation")
plot(tt,reshape(Pd(3,end,:),1,[]),'--',LineWidth=1,DisplayName="zd")
legend

figure(202)
subplot(3,1,1)
hold on
grid on
title("Simulated vs. teorical xdd")
xlabel("t [s]")
ylabel("acceleration [m/s^2]")
plot(simout.pdd.Time,simout.pdd.Data(:,1),LineWidth=1,DisplayName="xdd simulation")
plot(tt,reshape(Pdd(1,end,:),1,[]),'--',LineWidth=1,DisplayName="xdd")
legend

subplot(3,1,2)
hold on
grid on
title("Simulated vs. teorical ydd")
xlabel("t [s]")
ylabel("acceleration [m/s^2]")
plot(simout.pdd.Time,simout.pdd.Data(:,2),LineWidth=1,DisplayName="ydd simulation")
plot(tt,reshape(Pdd(2,end,:),1,[]),'--',LineWidth=1,DisplayName="ydd")
legend

subplot(3,1,3)
hold on
grid on
title("Simulated vs. teorical zdd")
xlabel("t [s]")
ylabel("acceleration [m/s^2]")
plot(simout.pdd.Time,simout.pdd.Data(:,3),LineWidth=1,DisplayName="zdd simulation")
plot(tt,reshape(Pdd(3,end,:),1,[]),'--',LineWidth=1,DisplayName="zdd")
legend

figure(203)
subplot(3,1,1)
hold on
grid on
title("Simulated vs. teorical q1")
xlabel("t [s]")
ylabel("position [rad]")
plot(simout.q.Time,simout.q.Data(:,1),LineWidth=1,DisplayName="q1 simulation")
plot(tt,Q(1,:),'--',LineWidth=1,DisplayName="q1")
legend

subplot(3,1,2)
hold on
grid on
title("Simulated vs. teorical q2")
xlabel("t [s]")
ylabel("position [rad]")
plot(simout.q.Time,simout.q.Data(:,2),LineWidth=1,DisplayName="q2 simulation")
plot(tt,Q(2,:),'--',LineWidth=1,DisplayName="q2")
legend

subplot(3,1,3)
hold on
grid on
title("Simulated vs. teorical q3")
xlabel("t [s]")
ylabel("position [rad]")
plot(simout.q.Time,simout.q.Data(:,3),LineWidth=1,DisplayName="q3 simulation")
plot(tt,Q(3,:),'--',LineWidth=1,DisplayName="q3")
legend

figure(204)
subplot(3,1,1)
hold on
grid on
title("Simulated vs. teorical qd1")
xlabel("t [s]")
ylabel("velocity [rad/s]")
plot(simout.qd.Time,simout.qd.Data(:,1),LineWidth=1,DisplayName="qd1 simulation")
plot(tt,Qd(1,:),'--',LineWidth=1,DisplayName="qd1")
legend

subplot(3,1,2)
hold on
grid on
title("Simulated vs. teorical qd2")
xlabel("t [s]")
ylabel("velocity [rad/s]")
plot(simout.qd.Time,simout.qd.Data(:,2),LineWidth=1,DisplayName="qd2 simulation")
plot(tt,Qd(2,:),'--',LineWidth=1,DisplayName="qd2")
legend

subplot(3,1,3)
hold on
grid on
title("Simulated vs. teorical qd3")
xlabel("t [s]")
ylabel("velocity [rad/s]")
plot(simout.qd.Time,simout.qd.Data(:,3),LineWidth=1,DisplayName="qd3 simulation")
plot(tt,Qd(3,:),'--',LineWidth=1,DisplayName="qd3")
legend

figure(205)
subplot(3,1,1)
hold on
grid on
title("Simulated vs. teorical qdd1")
xlabel("t [s]")
ylabel("acceleration [rad/s^2]")
plot(simout.qdd.Time,simout.qdd.Data(:,1),LineWidth=1,DisplayName="qdd1 simulation")
plot(tt,Qdd(1,:),'--',LineWidth=1,DisplayName="qdd1")
legend

subplot(3,1,2)
hold on
grid on
title("Simulated vs. teorical qdd2")
xlabel("t [s]")
ylabel("acceleration [rad/s^2]")
plot(simout.qdd.Time,simout.qdd.Data(:,2),LineWidth=1,DisplayName="qdd2 simulation")
plot(tt,Qdd(2,:),'--',LineWidth=1,DisplayName="qdd2")
legend

subplot(3,1,3)
hold on
grid on
title("Simulated vs. teorical qdd3")
xlabel("t [s]")
ylabel("acceleration [rad/s^2]")
plot(simout.qdd.Time,simout.qdd.Data(:,3),LineWidth=1,DisplayName="qdd3 simulation")
plot(tt,Qdd(3,:),'--',LineWidth=1,DisplayName="qdd3")
legend

figure(206)
subplot(3,1,1)
hold on
grid on
title("Simulated vs. teorical torque on joint 2")
xlabel("t [s]")
ylabel("torque [Nm]")
plot(simout.Fq.Time,simout.Fq.Data(:,1),LineWidth=1,DisplayName="theta2' simulation")
plot(tt,torques(:,1),'--',LineWidth=1,DisplayName="theta2'")
legend

subplot(3,1,2)
hold on
grid on
title("Simulated vs. teorical torque on joint 4")
xlabel("t [s]")
ylabel("torque [Nm]")
plot(simout.Fq.Time,simout.Fq.Data(:,2),LineWidth=1,DisplayName="theta4 simulation")
plot(tt,torques(:,2),'--',LineWidth=1,DisplayName="theta4")
legend

subplot(3,1,3)
hold on
grid on
title("Simulated vs. teorical torque on joint 5")
xlabel("t [s]")
ylabel("torque [Nm]")
plot(simout.Fq.Time,simout.Fq.Data(:,3),LineWidth=1,DisplayName="theta5 simulation")
plot(tt,torques(:,3),'--',LineWidth=1,DisplayName="theta5")
legend

figure(207)
hold on
plot3(reshape(P(1,end,:),1,[]), reshape(P(2,end,:),1,[]), reshape(P(3,end,:),1,[]), "--", LineWidth=1)
plot3(simout.p.Data(:,1), simout.p.Data(:,2), simout.p.Data(:,3), "-", LineWidth=1)
plot3(coords(1,:), coords(2,:), coords(3,:), "k*")
grid on
title('Working space trajectory')
xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")
legend("Calculated trajectory","Simulated trajectory", "Objective trajectory")
view(45,45)