
theta = [0, 3, 2, -1, -3, 2, 7, 5];
A = 1;
V = 5;
numpoints = 50;
[q, qd, qdd, tt, T, t] = lineparabolas(theta, A, V);

figure(1)
subplot(3,1,1)
hold on
plot(tt, q, 'r-')
plot(T, theta, 'k.')
for i = 1:length(T)
    if i == 1
        plot([T(i)+t(i), T(i)+(t(i))], [-10,10], 'k--', 'HandleVisibility', 'off')
    elseif i == length(T)
        plot([T(i)-t(i), T(i)-(t(i))], [-10,10], 'k--', 'HandleVisibility', 'off')
    else
        plot([T(i)+t(i)/2, T(i)+(t(i)/2)], [-10,10], 'k--', 'HandleVisibility', 'off')
        plot([T(i)-t(i)/2, T(i)+-(t(i)/2)], [-10,10], 'k--', 'HandleVisibility', 'off')
    end
end

grid on
legend('pos', 'nodes')

subplot(3,1,2)
hold on
plot(tt, qd, 'g-')
grid on
legend('vel')
for i = 1:length(T)
    if i == 1
        plot([T(i)+t(i), T(i)+(t(i))], [-10,10], 'k--', 'HandleVisibility', 'off')
    elseif i == length(T)
        plot([T(i)-t(i), T(i)-(t(i))], [-10,10], 'k--', 'HandleVisibility', 'off')
    else
        plot([T(i)+t(i)/2, T(i)+(t(i)/2)], [-10,10], 'k--', 'HandleVisibility', 'off')
        plot([T(i)-t(i)/2, T(i)+-(t(i)/2)], [-10,10], 'k--', 'HandleVisibility', 'off')
    end
end

subplot(3,1,3)
hold on
plot(tt, qdd, 'b-')
grid on
legend('acc')
for i = 1:length(T)
    if i == 1
        plot([T(i)+t(i), T(i)+(t(i))], [-10,10], 'k--', 'HandleVisibility', 'off')
    elseif i == length(T)
        plot([T(i)-t(i), T(i)-(t(i))], [-10,10], 'k--', 'HandleVisibility', 'off')
    else
        plot([T(i)+t(i)/2, T(i)+(t(i)/2)], [-10,10], 'k--', 'HandleVisibility', 'off')
        plot([T(i)-t(i)/2, T(i)+-(t(i)/2)], [-10,10], 'k--', 'HandleVisibility', 'off')
    end
end
