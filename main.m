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
Si = [-0.1 3.2 -0.2];
Sf = [0.1 3.6 0.2];

dT=1/100; % time step

% Calculate minimum actuation time
[t1, t3, T] = minactime(Sf(1)-Si(1),0.02,0.02,0.1,"ts");
ll1 = t1/T;
ll3 = t3/T;
tt= 0:dT:T;
n=length(tt);

Phie5 = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];

for i=1:n % generate motion in working space
    [ x(i), xd(i), xdd(i) ] = tretratti(tt(i),T,Si(1),Sf(1)-Si(1),ll1,ll3);
    [ y(i), yd(i), ydd(i) ] = tretratti(tt(i),T,Si(2),Sf(2)-Si(2),ll1,ll3);
    [ z(i), zd(i), zdd(i) ] = tretratti(tt(i),T,Si(3),Sf(3)-Si(3),ll1,ll3);
    % [ x(i), xd(i), xdd(i) ] = cycloidal(tt(i),T,Si(1),Sf(1)-Si(1));
    % [ y(i), yd(i), ydd(i) ] = cycloidal(tt(i),T,Si(2),Sf(2)-Si(2));
    % [ z(i), zd(i), zdd(i) ] = cycloidal(tt(i),T,Si(3),Sf(3)-Si(3));

    [Q(:,i), Qd(:,i), Qdd(:,i)]=invkin(L,[x(i); y(i); z(i)], [xd(i); yd(i); zd(i)], [xdd(i); ydd(i); zdd(i)]);

    [M, W, H, Labs] = dirkin(L, Q(:,i), Qd(:,i), Qdd(:,i));

    P(:,:,i) = M(:,:,6);
    Pd(:,:,i) = (W(:,:,6)*M(:,:,6));
    Pdd(:,:,i) = (H(:,:,6)*M(:,:,6));

    % Inertia moments calculations
    for j = 1:1:length(L)
        % calculate inertia moments with respect to the center of gravity
        if j==3 % link 3 is a special case

            % link l3'
            a = L(1)/2;
            b = L(1)/2;
            Jg = 1/12*m(1)*(a+b)^2;
            I3ig = [Jg 0 0; 0 0 0; 0 0 Jg];
            M33i = [1 0 0 -L(3)
                0 1 0 0
                0 0 1 0
                0 0 0 1];
            J3ig = pseudoinertia(I3ig, m(1), [0 0 0]);


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
            J(:,:,3) = M(:,:,4)*J(:,:,3)*M(:,:,4)';
        else
            a = L(j)/2;
            b = L(j)/2;
            m1 = m(j)/6;
            m2 = m(j)/6;
            mg = 2/3*m(j);
            Jg = 1/12*m(j)*(a+b)^2;

            I(:,:,j) = [0 0 0; 0 Jg 0; 0 0 Jg];
            G(:,:,j) = [-b 0 0];
            J(:,:,j) = pseudoinertia(I,m(j),G(:,:,j));
            J(:,:,j) = M(:,:,j+1)*J(:,:,j)*M(:,:,j+1);
            if j==2
                J1ii = M(:,:,7)*J(:,:,j)*M(:,:,7);
            end
        end
    end

    Phie0 = M(:,:,6)*Phie5*M(:,:,6)';
    [Phi, phi(i,:)] = invdyn(M,H,Labs,J,Phie0,L);

    % Kinetic and potential energy computation
    t(i) = 0;
    u (i) = 0;
    for l= 1:1:size(J,3)
        if l == 2
            t(i) = t(i) + 1/2*trace(W(:,:,7)*J(:,:,l)*W(:,:,7)');
        elseif l == 1
            t(i) = t(i) + 1/2*trace(W(:,:,l+2)*J(:,:,l)*W(:,:,l+2)');
        else
            t(i) = t(i) + 1/2*trace(W(:,:,l+1)*J(:,:,l)*W(:,:,l+1)');
        end
        u(i) = u(i) -trace(Hg*J(:,:,l));
    end
    p(i) = pseudot(Phi(:,:,1),W(:,:,3)) + pseudot(Phi(:,:,2),W(:,:,4)) + pseudot(Phi(:,:,4),W(:,:,5)) + pseudot(Phi(:,:,5),W(:,:,6));
end

plotmanipulator(Q(:,1), L, 'r', 1);
plotmanipulator(Q(:,end), L, 'b', 1);

figure(2)
hold on
plot(tt, x, ".-", LineWidth=1.5)
plot(tt, xd, ".-", LineWidth=1.5)
plot(tt, xdd, ".-", LineWidth=1.5)
plot(tt, reshape(P(1,end,:),1,[]), "-", LineWidth=1.2)
plot(tt, reshape(Pd(1,end,:),1,[]), "-", LineWidth=1.2)
plot(tt, reshape(Pdd(1,end,:),1,[]), "-", LineWidth=1.2)
% plot([0 T],[0 0], 'k')
grid on
title('x Linear')
legend('x','xd','xdd', 'xi','xdi','xddi')

figure(3)
hold on
plot(tt, y, ".-", LineWidth=1.5)
plot(tt, yd, ".-", LineWidth=1.5)
plot(tt, ydd, ".-", LineWidth=1.5)
plot(tt, reshape(P(2,end,:),1,[]), "-", LineWidth=1.2)
plot(tt, reshape(Pd(2,end,:),1,[]), "-", LineWidth=1.2)
plot(tt, reshape(Pdd(2,end,:),1,[]), "-", LineWidth=1.2)
% plot([0 T],[0 0], 'k')
grid on
title('Y Linear')
legend('y','yd','ydd', 'yi','ydi','yddi')

figure(4)
hold on
plot(tt, z, ".-", LineWidth=1.5)
plot(tt, zd, ".-", LineWidth=1.5)
plot(tt, zdd, ".-", LineWidth=1.5)
plot(tt, reshape(P(3,end,:),1,[]), "-", LineWidth=1.2)
plot(tt, reshape(Pd(3,end,:),1,[]), "-", LineWidth=1.2)
plot(tt, reshape(Pdd(3,end,:),1,[]), "-", LineWidth=1.2)
% plot([0 T],[0 0], 'k')
grid on
title('Z Linear')
legend('z','zd','zdd', 'zi','zdi','zddi')

figure(5)
hold on
plot(tt, phi(:,1))
plot(tt, phi(:,2))
plot(tt, phi(:,3))
% plot([0 T],[0 0], 'k')
grid on
title('Motor torques')
legend('theta2''','theta4','theta5')

figure(6)
hold on
plot(tt, t)
plot(tt, u)
grid on
title('Energy')
legend('Ek', 'Ep')

de = diff(t+u)/dT;

figure(7)
hold on
plot(tt(2:end), de)
plot(tt, p)
grid on
title('Power')
legend('d(T+U)/dt', 'W')

figure(8)
hold on
plot(tt, Q(1,:), "-", LineWidth=1.5)
plot(tt, Qd(1,:), "-", LineWidth=1.5)
plot(tt, Qdd(1,:), "-", LineWidth=1.5)
% plot([0 T],[0 0], 'k')
grid on
title('x Linear')
legend('x','xd','xdd')

figure(9)
hold on
plot(tt, Q(2,:), "-", LineWidth=1.5)
plot(tt, Qd(2,:), "-", LineWidth=1.5)
plot(tt, Qdd(2,:), "-", LineWidth=1.5)
% plot([0 T],[0 0], 'k')
grid on
title('Y Linear')
legend('y','yd','ydd')

figure(10)
hold on
plot(tt, Q(3,:), "-", LineWidth=1.5)
plot(tt, Qd(3,:), "-", LineWidth=1.5)
plot(tt, Qdd(3,:), "-", LineWidth=1.5)
% plot([0 T],[0 0], 'k')
grid on
title('Z Linear')
legend('z','zd','zdd')
