close all

letterX = [-1,1.5,1.8,2,1.8,1.5,1,0.5,0.2,0,0.2,0.5,1.5,1.8,1.9,1.8,1.5];
letterY = [3,3,2.8,2.5,2.2,2,1.9,2,2.2,2.5,2.8,3,3,3.1,3.2,3.4,3.5];

% Place the trajectory on an inclined plane
a = .7; b = .7; d=-2.2; 
Z = @(x,y) a*x + b*y +d;
letterZ = Z(letterX, letterY);

theta = [letterX; letterY; letterZ];

A = [.1, 0.2, 0.05]';
V = [.5, .7, .6]';
numpoints = 50;
[q, qd, qdd, tt, T, t] = lineparabolas2(theta, A, V);

d = height(theta);

%% -------------------------- PLOT ACTUATORS -------------------------- %%

parfor j = 1:d
    figure(j)
    subplot(3,1,1)
    hold on
    plot(tt, q(j,:), 'r-')
    plot(T, theta(j,:), 'k.')
    for i = 1:length(T)
        if i == 1
            plot([T(i)+t(i), T(i)+(t(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        elseif i == length(T)
            plot([T(i)-t(i), T(i)-(t(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        else
            plot([T(i)+t(i)/2, T(i)+(t(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
            plot([T(i)-t(i)/2, T(i)+-(t(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        end
    end
    ylim([min(q(j,:)), max(q(j,:))])

    grid on
    legend('pos', 'nodes')

    subplot(3,1,2)
    hold on
    plot(tt, qd(j,:), 'g-')
    plot([tt(1), tt(end)], [V(j), V(j)], 'r--')
    plot([tt(1), tt(end)], -[V(j), V(j)], 'r--')
    grid on
    legend('vel')
    for i = 1:length(T)
        if i == 1
            plot([T(i)+t(i), T(i)+(t(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        elseif i == length(T)
            plot([T(i)-t(i), T(i)-(t(i))], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        else
            plot([T(i)+t(i)/2, T(i)+(t(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
            plot([T(i)-t(i)/2, T(i)+-(t(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        end
    end
    ylim([-V(j), V(j)])

    subplot(3,1,3)
    hold on
    plot(tt, qdd(j,:), 'b-')
    plot([tt(1), tt(end)], [A(j), A(j)], 'r--')
    plot([tt(1), tt(end)], -[A(j), A(j)], 'r--')
    grid on
    legend('acc')
    for i = 1:length(T)
        if i == 1
            plot([T(i)+t(i), T(i)+t(i)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        elseif i == length(T)
            plot([T(i)-t(i), T(i)-t(i)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        else
            plot([T(i)+t(i)/2, T(i)+(t(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
            plot([T(i)-t(i)/2, T(i)+-(t(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        end
    end
    ylim([-A(j), A(j)])

end

%% -------------------------- PLOT COMPARISON -------------------------- %%

figure(100)

parfor j = 1:d
    subplot(d,1,j)
    hold on
    grid on
    plot(tt, q(j,:), 'r-')
    plot(T, theta(j,:), 'k.')
    for i = 1:length(T)
        if i == 1
            plot([T(i)+t(i), T(i)+t(i)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        elseif i == length(T)
            plot([T(i)-t(i), T(i)-t(i)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        else
            plot([T(i)+t(i)/2, T(i)+(t(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
            plot([T(i)-t(i)/2, T(i)+-(t(i)/2)], [-1000,1000], 'k--', 'HandleVisibility', 'off')
        end
    end
    ylim([min(q(j,:)), max(q(j,:))])
end
%% -------------------- PLOT TRAJECTORY IN SPACE ----------------------- %%
figure(200)
plot3(q(1,:), q(2,:), q(3,:), "b-");
xlabel("X");
ylabel("Y");
zlabel("Z");
grid on
