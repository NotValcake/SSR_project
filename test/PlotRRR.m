function PlotRRR(Q,L,colore,fig)
% routine per plottare lo schema del robot in una certa configurazione
figure(fig)
q1 = Q(1);
q2 = Q(2);
q3 = Q(3);
h = L(1);
l1 = L(2);
l2 = L(3);
l3 = L(4);
x1 = 0;
y1 = 0;
z1 = 0;
x2 = 0;
y2 = 0;
z2 = h;
x3 = l1*cos(q1);
y3 = l1*sin(q1);
z3 = h;
x4 = l1*cos(q1)+l2*cos(q1+q2);
y4 = l1*sin(q1)+l2*sin(q1+q2);
z4 = h;
x5 = l1*cos(q1)+(l2+l3*cos(q3))*cos(q1+q2);
y5 = l1*sin(q1)+(l2+l3*cos(q3))*sin(q1+q2);
z5 = h+l3*sin(q3);
plot3([0 x1 x2 x3 x4 x5],[0 y1 y2 y3 y4 y5],[0 z1 z2 z3 z4 z5],'LineWidth',2,'color',colore);% polot manipulator
plot3([0], [0], [0],'*','color','k');%base
plot3([x2 x3 x4],[y2 y3 y4],[z2 z3 z4], 'o','color',colore); %joint
position
