function Je = fourbarjacext(L,Q)
%FOURBARJACEXT extended jacobian of a four bar linkage
Je(1)=0; % x1
Je(2)=0; % y1
Je(3)=0; %xg1
Je(4)=0; %yg1
Je(5)=-L(2)*sin(Q);%x2
Je(6)=L(2)*cos(Q);%y2
Je(7)=-L(2)/2*sin(Q);%xg2
Je(8)=L(2)/2*cos(Q);%yg2
Je(9)=-L(2)*sin(Q);%x3
Je(10)=L(2)*cos(Q);%y3
Je(11)=-L(2)/2*sin(Q);%xg3
Je(12)=L(2)/2*cos(Q);%yg3
Je(13)=Je(5);%xg4
Je(14)=Je(6);%yg4
Je(15)=1;%alpha
Je(16)=Je(13);%x3
Je(17)=Je(14);%yg3
Je=Je';
end

