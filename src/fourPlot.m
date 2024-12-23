function [] = fourPlot(x, u, px, acceleration, Offset, time)

subplot(4, 1, 1);
plot(time(2:(end-Offset)), x.v_h(2:(end-Offset)), time(2:(end-Offset)), px.v_p(2:(end-Offset)),'LineWidth', 2);
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Vehicle Speed over Time');
grid on;
legend('Host Vehicle', 'Lead Vehicle')
subplot(4, 1, 2);
plot(time(2:(end-Offset)), x.s_h(2:(end-Offset)), time(2:(end-Offset)), px.s_p(2:(end-Offset)), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position (m)');
title('Vehicle Position over Time');
grid on;

subplot(4, 1, 3);
plot(time(2:(end-Offset)), acceleration(2:(end-Offset)),'LineWidth', 2);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Vehicle Acceleration over Time');
grid on;

subplot(4, 1, 4);
plot(time(2:(end-Offset)), x.n_g(2:(end-Offset)), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Gear Position');
title('Gear Position over Time');
grid on;
end