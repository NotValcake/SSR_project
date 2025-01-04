
  % % Define the lengths of each link
  % L = [0.5, 0.3, 0.4, 0.2, 0.1];

  % Define the lengths of each link
  L = [1, 1, 1, 1, 1];

  % Define the joint configurations (angles in radians, velocities, and accelerations)
  Q = [deg2rad(45), deg2rad(60), deg2rad(125)];
  Qd = [0.1, 0.2, 0.3];
  Qdd = [0.01, 0.02, 0.03];

  plotmanipulator2(Q,L,'r',1)   
  legend ('Links','','','Joints','End-effector')
  hold on

  % Calculate end-effector position, velocity, and acceleration
  [M, Wabs, Habs, Labs] = dirkin(L, Q, Qd, Qdd);

  % Calculate gripper position, velocity and acceleration
  P = M(:,:,6)
  Pd = Wabs(:,:,6)*P
  Pdd = Habs(:,:,6)*P

  plotareaxy2(L,2)
  saveas(gcf, "../Img/areaxy.jpg")
  plotareaxz2(L,6)
  saveas(gcf, "../Img/areaxz.jpg")
  plotareayz2(L,4)
  saveas(gcf, "../Img/areayz.jpg")

  % Test inverse kinematic
  [Qi, Qdi, Qddi] = invkin(L,P(1:3,end),Pd(1:3,end),Pdd(1:3,end))
  Qi = mod(Qi,2*pi);
  [Mi, Wabsi, Habsi, Labsi] = dirkin(L, Qi, Qdi, Qddi);

  % Calculate gripper position, velocity and acceleration
  Pi = Mi(:,:,6)
  Pdi = Wabsi(:,:,6)*P
  Pddi = Habsi(:,:,6)*P

  plotmanipulator2(Q,L,'g',1)   
  legend('Links','','','Joints','End-effector')
  
  %% Test inverse dynamics for links 6-5
  m = [1 1 1 1 1];
  for i = 1:1:length(L)
      I(:,:,i) = [1 0 0 0.5; 0 1 0 0; 0 0 1 0; 0.5 0 0 0.5];
      G(:,:,i) = [0 0 0 0.5; 0 0 0 0; 0 0 0 0; 0 0 0 0];
  end
  
  Phi = invdyn(Mi,Habs,Labs,I,m,G,[0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0]);