  
% Define the lengths of each link
L = [1, 1, 1, 1, 1];

% Jacobian matrix
J = @(Q,L) [
        -L(2) * sin(Q(1)),                     0,                    -L(5) * cos(Q(3));
         L(2) * cos(Q(1)), -sin(Q(2)) * (L(4) + L(5) * cos(Q(3))),   -L(5) * cos(Q(2)) * sin(Q(3));
                       0, -cos(Q(2)) * (L(4) + L(5) * cos(Q(3))),    L(5) * sin(Q(2)) * sin(Q(3))];

% Define the joint configurations (angles in radians, velocities, and accelerations)
Q(:,1) = [0, pi/2, pi/4];
Q(:,2) = [0, pi/4, pi/2];
Q(:,3) = [pi/2, pi/4, 0];
Q(:,4) = [pi/4, pi/2, 0];
Q(:,5) = [pi/4, pi/4, acos(-L(4)/L(5))];
Q(:,6) = [pi/4, pi/4, pi/4];

for idx = 1:6
    hold on
    % figure(1)
    % subplot(3,2,idx)
    [xtcp, ytcp, ztcp] = plotmanipulator2(Q(:,idx),L,'r',idx);
    % [xtcp, ytcp, ztcp] = plotmanipulator(Q(:,idx),L,'r',idx);
    %  plotellipsoid(xtcp, ytcp, ztcp, J(Q(:,idx),L))
    title("\theta_{2'}=" + Q(1,idx) + ", \theta_{4}=" +Q(2,idx) + ", \theta_{5}=" + Q(3,idx))
    fontsize(16,"points")

    % axis equal
    % xlim([0 2])
    % ylim([0 4])
    % zlim([-2 2])
    plot3([-10,10],[0,0],[0,0],'k')
    plot3([0,0],[-10,10],[0,0],'k')
    plot3([0,0],[0,0],[-10,10],'k')

    % limits to be used with plotmanipulator2 results
    xlim([-2 3])
    ylim([0 4])
    zlim([-3 3])
    legend('Links','','Base link','Joints','End-effector', '')

    % limits to be used with plotmanipulator results
    % xlim([-2 3])
    % ylim([-1 6])
    % zlim([-3 3])
    % legend('Links','','Base link','Joints','End-effector', 'Man. Ellipsoid')

    saveas(gcf, "../Img/singularity_"+idx+".jpg")
end