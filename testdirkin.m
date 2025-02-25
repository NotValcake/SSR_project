
  % % Define the lengths of each link
  % L = [0.5, 0.3, 0.4, 0.2, 0.1];

  % Define the lengths of each link
  L = [1, 1, 1, 1, 1];

  % Define the joint configurations (angles in radians, velocities, and accelerations)
  Q = [deg2rad(45), deg2rad(60), deg2rad(125)];
  % Test a singular configuration
  % Q = [deg2rad(180), deg2rad(90), deg2rad(125)];
  Qd = [0.1, 0.2, 0.3];
  Qdd = [0.01, 0.02, 0.03];

  plotmanipulator2(Q,L,'r',1);
  legend ('Links','','','Joints','End-effector')
  hold on

  % Calculate end-effector position, velocity, and acceleration
  [M, Wabs, Habs, Labs] = dirkin(L, Q, Qd, Qdd);

  % Calculate gripper position, velocity and acceleration
  P = M(:,:,6);
  Pd = Wabs(:,:,6)*P;
  Pdd = Habs(:,:,6)*P;
  
  % Workspace plot
  plotareaxy2(L,2)
  % saveas(gcf, "../Img/areaxy.jpg")
  plotareaxz2(L,6)
  % saveas(gcf, "../Img/areaxz.jpg")
  plotareayz2(L,4)
  % saveas(gcf, "../Img/areayz.jpg")

  % Test inverse kinematic
  [Qi, Qdi, Qddi] = invkin(L,P(1:3,end),Pd(1:3,end),Pdd(1:3,end));
  Qi = mod(Qi,2*pi);
  [Mi, Wabsi, Habsi, Labsi] = dirkin(L, Qi, Qdi, Qddi);

  % Calculate gripper position, velocity and acceleration
  Pi = Mi(:,:,6);
  Pdi = Wabsi(:,:,6)*Pi;
  Pddi = Habsi(:,:,6)*Pi;

  plotmanipulator2(Q,L,'g',1);   
  legend('Links','','','Joints','End-effector')
  
  % Test inverse dynamics for links 6-5
  % Mass of each link
  m = [1 1 1 1 1];

  % Inertia tensor for each link, using the equivalent 3 mass approximation
  % initialization of inertia matrix
  I (:,:,1) = zeros(3,3);
  G (:,:,1) = zeros(1,3);
  J (:,:,1) = zeros(4,4);

  for i = 1:1:length(L)
      % calculate inertia moments with respect to the center of gravity
      if i==3 % link 3 is a special case

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
          a = L(i)/2;
          b = L(i)/2;
          m1 = m(i)/6;
          m2 = m(i)/6;
          mg = 2/3*m(i);
          Jg = 1/12*m(i)*(a+b)^2;

          I(:,:,i) = [0 0 0; 0 Jg 0; 0 0 Jg];
          G(:,:,i) = [-b 0 0];
          J(:,:,i) = pseudoinertia(I,m(i),G(:,:,i));
          J(:,:,i) = M(:,:,i+1)*J(:,:,i)*M(:,:,i+1);
          if i==2
              J1ii = M(:,:,7)*J(:,:,i)*M(:,:,7);
          end
      end
  end

  Phie5 = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
  Phie0 = M(:,:,6)*Phie5*M(:,:,6)';
  [Phi, phi] = invdyn(M,Habs,Labs,J,Phie0,L);

  Hg = [0, 0, 0, -9.81;  % Gravitational acceleration matrix
          0, 0, 0, 0;
          0, 0, 0, 0;
          0, 0, 0, 0];
  
  %% Setup for Lgrange

  % rearrange conveniently terms to apply lagrangian formulation
  % Qdd = [theta2', theta1'', theta3', theta3'', theta4, theta5]
  Qdd = [Qdd(1), Qdd(1), -Qdd(1), -Qdd(1), Qdd(2), Qdd(3)];
  L = cat(3, Labs(:,:,2), Labs(:,:,1), Labs(:,:,3), Labs(:,:,7), Labs(:,:,4), Labs(:,:,5));
  
  % Bodies variable are arranged [l2', l1', l1'', l3, l4, l5]
  W = cat(3, Wabs(:,:,3), Wabs(:,:,7), Wabs(:,:,4), Wabs(:,:,5), Wabs(:,:,6));
  Jabs = cat(3, J(:,:,2), J1ii, J(:,:,3), J(:,:,4), J(:,:,5));

  Qm = lagrange(W, Jabs, L, Qdd, Hg); 

  % Calcolo potenza
  % Ec = 0;
  % Ep = 0;
  % for i = 1:1:length(L)
  %     Ec = Ec + 1/2*trace(Wabsi(:,:,i)*pseudoinertia(I(:,:,i),m(i),G(:,:,i))*Wabsi(:,:,i)');
  %     Ep = Ep + trace(-Hg*pseudoinertia(I(:,:,i),m(i),G(:,:,i)));
  % end
  % 
  % power1 = diff(Ec+Ep);
  % power2 = [Phi(1), Phi(3), Phi(4)]*Qdd';