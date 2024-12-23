function [] = comparePlot(x1,x2,x3,u1,u2,u3,f1,f2,f3,time,Offset)
    subplot(3, 1, 1);
    plot(time(2:(end-Offset)), u1.T_f(2:(end-Offset)),time(2:(end-Offset)), u2.T_f(2:(end-Offset)),time(2:(end-Offset)), u3.T_f(2:(end-Offset)),'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Engine Torque (Nm)');
    title('(a)');
    grid on;

    subplot(3, 1, 2);
    plot(time(2:(end-Offset)), x1.n_g(2:(end-Offset)),time(2:(end-Offset)), x2.n_g(2:(end-Offset)),time(2:(end-Offset)), x3.n_g(2:(end-Offset)),'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Gear Position)');
    title('(b)');
    grid on;
    subplot(3, 1, 3);

    plot(time(2:(end-Offset)), f1(2:(end-Offset)),time(2:(end-Offset)), f2(2:(end-Offset)),time(2:(end-Offset)), f3(2:(end-Offset)),'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Total Fuel Consumption (kg)');
    title('(c)');
    grid on;
    legend('PMP-Bisection','IP','SQP');
end