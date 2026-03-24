function oldPlotGraph(reference_signal,...
    non_linear_full_states,...
    per_motor_thrust, ...
    partial_states_estimates, ...
    measurable_states, ...
    pathFG_time, ...
    MPC_time)

    % pos=[left, bottom, width, height]
    % subplot(rows, columns, figure_number)
    fig1 = figure('pos', [1220 50 1200 800]);
    subplot(7,1,1), plot(reference_signal.Time, reference_signal.Data(:, 1),'r','LineWidth', 3), hold on;
    subplot(7,1,1), plot(non_linear_full_states.Time, non_linear_full_states.Data(:,1),'b','LineWidth', 3), hold on
    ylabel('x [m]','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',12,'fontweight','bold')
    legend('Reference: x','Nonlinear: x','FontSize', 12)
    % ylim([2, 6]); % Set the limits for the y-axis
    
    subplot(7,1,2), plot(reference_signal.Time, reference_signal.Data(:, 2),'r','LineWidth', 3), hold on;
    subplot(7,1,2), plot(non_linear_full_states.Time, non_linear_full_states.Data(:,2),'b','LineWidth', 3), hold on
    ylabel('y [m]','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',12,'fontweight','bold')
    legend('Reference: y','Nonlinear: y','FontSize', 12)
    % ylim([2, 6]); % Set the limits for the y-axis
    
    subplot(7,1,3), plot(reference_signal.Time, reference_signal.Data(:, 3),'r','LineWidth', 3), hold on;
    subplot(7,1,3), plot(non_linear_full_states.Time, non_linear_full_states.Data(:, 3),'b','LineWidth', 3), hold on
    ylabel('z [m]','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',12,'fontweight','bold')
    legend('Reference: z','Nonlinear: z','FontSize', 12)
    % ylim([2, 6]); % Set the limits for the y-axis
    
    subplot(7,1,4), plot(non_linear_full_states.Time, non_linear_full_states.Data(:, 7),'b','LineWidth', 3), hold on
    ylabel('roll [rad]','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',12,'fontweight','bold')
    legend('Nonlinear: roll angle','FontSize', 12)% xlim([0, 10]); % Set the limits for the x-axis
    % ylim([-500, 500]); % Set the limits for the y-axis
    
    subplot(7,1,5), plot(non_linear_full_states.Time, non_linear_full_states.Data(:, 8),'b','LineWidth', 3), hold on
    ylabel('pitch [rad]','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',12,'fontweight','bold')
    legend('Nonlinear: pitch angle','FontSize', 12)
    
    subplot(7,1,6), plot(reference_signal.Time, reference_signal.Data(:, 4),'r','LineWidth', 3), hold on;
    subplot(7,1,6), plot(non_linear_full_states.Time, non_linear_full_states.Data(:, 9),'b','LineWidth', 3), hold on
    ylabel('yaw [rad]','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',12,'fontweight','bold')
    legend('Nonlinear: yaw angle','FontSize', 12)
    
    subplot(7,1,7), plot(per_motor_thrust.Time, per_motor_thrust.Data(:, 1),'r','LineWidth', 2), hold on;
    subplot(7,1,7), plot(per_motor_thrust.Time, per_motor_thrust.Data(:, 2),'b','LineWidth', 2), hold on;
    subplot(7,1,7), plot(per_motor_thrust.Time, per_motor_thrust.Data(:, 3),'y','LineWidth', 2), hold on;
    subplot(7,1,7), plot(per_motor_thrust.Time, per_motor_thrust.Data(:, 4),'g','LineWidth', 2), hold on;
    ylabel('thrusts [N]','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',12,'fontweight','bold')
    legend('Thrust 1','Thrust 2','Thrust 3','Thrust 4','FontSize', 12)
    % xlim([0, 5]); % Set the limits for the x-axis
    % ylim([0.07, 0.9]); % Set the limits for the y-axis
    
    % set(gcf, 'WindowState', 'maximized');

    fig2 = figure('pos', [1250 50 1200 800]);
    subplot(9,1,1), plot(measurable_states.Time, measurable_states.Data(1, :),'g','LineWidth', 0.05), hold on
    subplot(9,1,1), plot(partial_states_estimates.Time, partial_states_estimates.Data(1, :),'r','LineWidth', 3), hold on;
    subplot(9,1,1), plot(non_linear_full_states.Time, non_linear_full_states.Data(:,1),'b','LineWidth', 3), hold on
    ylabel('x [m]','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',10,'fontweight','bold')
    legend('measurement: x', 'Estimate: x','Nonlinear: x','FontSize', 10)
   
    subplot(9,1,2), plot(measurable_states.Time, measurable_states.Data(2, :),'g','LineWidth', 0.05), hold on
    subplot(9,1,2), plot(partial_states_estimates.Time, partial_states_estimates.Data(2,:),'r','LineWidth', 3), hold on;
    subplot(9,1,2), plot(non_linear_full_states.Time, non_linear_full_states.Data(:,2),'b','LineWidth', 3), hold on
    ylabel('y [m]','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',10,'fontweight','bold')
    legend('measurement: y','Estimate: y','Nonlinear: y','FontSize', 10)

    subplot(9,1,3), plot(measurable_states.Time, measurable_states.Data(3, :),'g','LineWidth', 0.05), hold on
    subplot(9,1,3), plot(partial_states_estimates.Time, partial_states_estimates.Data(3,:),'r','LineWidth', 3), hold on;
    subplot(9,1,3), plot(non_linear_full_states.Time, non_linear_full_states.Data(:,3),'b','LineWidth', 3), hold on
    ylabel('z [m]','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',10,'fontweight','bold')
    legend('measurement: z','Estimate: z','Nonlinear: z','FontSize', 10)

    subplot(9,1,4), plot(measurable_states.Time, measurable_states.Data(4, :),'g','LineWidth', 3), hold on
    subplot(9,1,4), plot(partial_states_estimates.Time, partial_states_estimates.Data(4,:),'r','LineWidth', 3), hold on;
    subplot(9,1,4), plot(non_linear_full_states.Time, non_linear_full_states.Data(:,4),'b','LineWidth', 3), hold on
    ylabel('xdot','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',10,'fontweight','bold')
    legend('measurement: xdot','Estimate: xdot','Nonlinear: xdot','FontSize', 10)

    subplot(9,1,5), plot(measurable_states.Time, measurable_states.Data(5, :),'g','LineWidth', 3), hold on
    subplot(9,1,5), plot(partial_states_estimates.Time, partial_states_estimates.Data(5,:),'r','LineWidth', 3), hold on;
    subplot(9,1,5), plot(non_linear_full_states.Time, non_linear_full_states.Data(:,5),'b','LineWidth', 3), hold on
    ylabel('ydot','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',10,'fontweight','bold')
    legend('measurement: ydot','Estimate: ydot','Nonlinear: ydot','FontSize', 10)

    subplot(9,1,6), plot(measurable_states.Time, measurable_states.Data(6, :),'g','LineWidth', 3), hold on
    subplot(9,1,6), plot(partial_states_estimates.Time, partial_states_estimates.Data(6,:),'r','LineWidth', 3), hold on;
    subplot(9,1,6), plot(non_linear_full_states.Time, non_linear_full_states.Data(:,6),'b','LineWidth', 3), hold on
    ylabel('zdot','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',10,'fontweight','bold')
    legend('measurement: zdot','Estimate: zdot','Nonlinear: zdot','FontSize', 10)

    subplot(9,1,7), plot(measurable_states.Time, measurable_states.Data(7, :),'g','LineWidth', 3), hold on
    subplot(9,1,7), plot(partial_states_estimates.Time, partial_states_estimates.Data(7,:),'r','LineWidth', 3), hold on;
    subplot(9,1,7), plot(non_linear_full_states.Time, non_linear_full_states.Data(:,7),'b','LineWidth', 3), hold on
    ylabel('roll','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',10,'fontweight','bold')
    legend('measurement: roll','Estimate: roll','Nonlinear: roll','FontSize', 10)

    subplot(9,1,8), plot(measurable_states.Time, measurable_states.Data(8, :),'g','LineWidth', 3), hold on
    subplot(9,1,8), plot(partial_states_estimates.Time, partial_states_estimates.Data(8,:),'r','LineWidth', 3), hold on;
    subplot(9,1,8), plot(non_linear_full_states.Time, non_linear_full_states.Data(:,8),'b','LineWidth', 3), hold on
    ylabel('pitch','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',10,'fontweight','bold')
    legend('measurement: pitch','Estimate: pitch','Nonlinear: pitch','FontSize', 10)

    subplot(9,1,9), plot(measurable_states.Time, measurable_states.Data(9, :),'g','LineWidth',3), hold on
    subplot(9,1,9), plot(partial_states_estimates.Time, partial_states_estimates.Data(9,:),'r','LineWidth', 3), hold on;
    subplot(9,1,9), plot(non_linear_full_states.Time, non_linear_full_states.Data(:,9),'b','LineWidth', 3), hold on
    ylabel('yaw','fontsize',12,'fontweight','bold')
    xlabel('Time [s]','fontsize',10,'fontweight','bold')
    legend('measurement: yaw','Estimate: yaw','Nonlinear: yaw','FontSize', 10)

    fig3 = figure(Position=[50, 1000, 1200, 600], Name='Computation Times');
    end_time = pathFG_time.Time(end);
    plot(pathFG_time.Time, pathFG_time.Data + MPC_time.Data, 'b'), hold on;
    plot(pathFG_time.Time, MPC_time.Data, 'r'), hold on;
    plot([0, end_time], mean(MPC_time.Data) * [1, 1], 'g'), hold on;
    plot([0, end_time], mean(pathFG_time.Data) * [1, 1], Color='black'), hold on;
    ylabel('Computation Time', 'FontSize', 12);
    xlabel('Time [s]', 'FontSize', 12);
    legend('pathFG', 'MPC', 'MPC Average', 'pathFG Average');

    % % export
    % saveas(fig1, 'figure1.pdf');
    % saveas(fig2, 'figure2.pdf');
    % saveas(fig3, 'figure3.pdf');

end