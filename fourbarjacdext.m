function Jed = fourbarjacdext(L,Q,Qp)
%FOURBARJACPEXT Summary of this function goes here
Jed(1)=0; % x1
Jed(2)=0; % y1
Jed(3)=0; %xg1
Jed(4)=0; %yg1
Jed(5)=-L(2)*cos(Q)*Qp;%x2
Jed(6)=-L(2)*sin(Q)*Qp;%y2
Jed(7)=-L(2)/2*cos(Q)*Qp;%xg2
Jed(8)=-L(2)/2*sin(Q)*Qp;%yg2
Jed(9)=-L(2)*cos(Q)*Qp;%x3
Jed(10)=-L(2)*sin(Q)*Qp;%y3
Jed(11)=-L(2)/2*cos(Q)*Qp;%xg3
Jed(12)=-L(2)/2*sin(Q)*Qp;%yg3
Jed(13)=Jed(5);%xg4
Jed(14)=Jed(6);%yg4
Jed(15)=0;%alpha
Jed(16)=Jed(13);%x3
Jed(17)=Jed(14);%yg3
Jed = Jed';
end

