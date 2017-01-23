function plot_sim_results(simulation_log, vehicle_parameters, name)
    % generate_Lat._controller Plot simulation results
    %    Inputs: simulation_log - Simulink output file
    %            vehicle_parameters - Vehicle parameter structure
    %                vehicle_parameters.m  - Vehicle mass [Kg]
    %                vehicle_parameters.l  - Wheelbase [m]
    %                vehicle_parameters.a  - Distance from cg to front axle (x) [m]
    %                vehicle_parameters.h  - Distance from cg to ground (z) [m]
    %                vehicle_parameters.Iz - Yaw moment of inertia [Kgm^2/rad]
    %                vehicle_parameters.g  - Gravity acceleration [m/s^2]
    %                vehicle_parameters.front_tire
    %                    front_tire.C  - Cornering stiffness [N/rad]
    %                    front_tire.Ru - Radio between frictions [-]
    %                vehicle_parameters.rear_tire
    %                    rear_tire.C  - Cornering stiffness [N/rad]
    %                    rear_tire.Ru - Radio between frictions [-]
    %            name - Filename where figure is saved
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    textwidth = 17.78;
    scale = 300 / get(0,'ScreenPixelsPerInch');

    t = simulation_log.get('chassis_state').Values.Time;

    chassis_x     = simulation_log.get('chassis_state').Values.Data;
    chassis_dx    = simulation_log.get('chassis_state_derivative').Values.Data;
    chassis_u     = simulation_log.get('chassis_control_inputs').Values.Data;
    chassis_mu    = simulation_log.get('chassis_friction_coeffs').Values.Data;
    chassis_kappa = simulation_log.get('chassis_curvature').Values.Data;
    
    has_observer = strcmp(simulation_log.getElementNames{end},'observer_state');
    
    if has_observer
        observer_t = simulation_log.get('observer_state').Values.Time;
        observer_x = simulation_log.get('observer_state').Values.Data;
        observer_u = simulation_log.get('observer_control_input').Values.Data;
    end

    fig = figure(1);
    clf

    % Crosstrack error plot
    subplot(3,3,1)
    grid on;
    hold on;
    if has_observer
        plot(observer_t, observer_x(:,2), 'r--')
    end
    plot(t, chassis_x(:,2), 'b')
    plot(t, - 0.4 * ones(size(t)), 'k-.')
    plot(t, 0.4 * ones(size(t)), 'k-.')
    set(gca, 'FontSize', scale*4)
    xlabel('Time - t [s]', 'FontSize', scale*8)
    ylabel('Crosstrack error - e_y [m]', 'FontSize', scale*8)
    xlim([0, max(t)])

    % Heading error plot
    subplot(3,3,2)
    grid on;
    hold on;
    if has_observer
        plot(observer_t, observer_x(:,3) * 180 / pi, 'r--')
    end
    plot(t, chassis_x(:,3) * 180 / pi, 'b')
    plot(t, - 18 * ones(size(t)), 'k-.')
    plot(t, 18 * ones(size(t)), 'k-.')
    set(gca, 'FontSize', scale*4)
    xlabel('Time - t [s]', 'FontSize', scale*8)
    ylabel('Heading error - e_\psi [\circ]', 'FontSize', scale*8)
    xlim([0, max(t)])

    % Long. velocity plot
    subplot(3,3,3)
    grid on;
    hold on;
    if has_observer
        plot(observer_t, observer_x(:,4), 'r--')
    end
    plot(t, chassis_x(:,4), 'b')
    % plot(t, - 18 * ones(size(t)), 'k-.')
    % plot(t, 18 * ones(size(t)), 'k-.')
    set(gca, 'FontSize', scale*4)
    xlabel('Time - t [s]', 'FontSize', scale*8)
    ylabel('Long. velocity - v_x [m/s]', 'FontSize', scale*8)
    xlim([0, max(t)])

    % Lat. velocity plot
    subplot(3,3,4)
    grid on;
    hold on;
    if has_observer
        plot(observer_t, observer_x(:,5), 'r--')
    end
    plot(t, chassis_x(:,5), 'b')
    % plot(t, - 18 * ones(size(t)), 'k-.')
    % plot(t, 18 * ones(size(t)), 'k-.')
    set(gca, 'FontSize', scale*4)
    xlabel('Time - t [s]', 'FontSize', scale*8)
    ylabel('Lat. velocity - v_y [m/s]', 'FontSize', scale*8)
    xlim([0, max(t)])

    % Yaw rate plot
    mu = min(chassis_mu, [], 2);
    r_max = mu * vehicle_parameters.world.g ./ chassis_x(:,4);

    subplot(3,3,5)
    grid on;
    hold on;
    if has_observer
        plot(observer_t, observer_x(:,6) * 180 / pi, 'r--')
    end
    plot(t, chassis_x(:,6) * 180 / pi, 'b')
    plot(t, - r_max * 180 / pi, 'k-.')
    plot(t, r_max * 180 / pi, 'k-.')
    set(gca, 'FontSize', scale*4)
    xlabel('Time - t [s]', 'FontSize', scale*8)
    ylabel('Yaw rate - r [\circ/s]', 'FontSize', scale*8)
    xlim([0, max(t)])

    % Steering angle plot
    subplot(3,3,6)
    grid on;
    hold on;
    if has_observer
        plot(observer_t, observer_u(:,1) * 180 / pi, 'r--')
    end
    plot(t, chassis_u(:,1) * 180 / pi, 'b')
    plot(t, - 24 * ones(size(t)), 'k-.')
    plot(t, 24 * ones(size(t)), 'k-.')
    set(gca, 'FontSize', scale*4)
    xlabel('Time - t [s]', 'FontSize', scale*8)
    ylabel('Steering angle - \delta_f [\circ]', 'FontSize', scale*8)
    xlim([0, max(t)])
    
    subplot(3,3,7)
    grid on;
    hold on;
    plot(t, chassis_kappa)
    set(gca, 'FontSize', scale*4)
    xlabel('Time - t [s]', 'FontSize', scale*8)
    ylabel('Curvature - \kappa [1/m]', 'FontSize', scale*8)

    % Slip Envelopes
    a = vehicle_parameters.body.a;
    b = vehicle_parameters.body.l - vehicle_parameters.body.a;
    alpha_f = atan2(chassis_x(:,5) + a * chassis_x(:,6), chassis_x(:,4)) - chassis_u(:,1);
    alpha_r = atan2(chassis_x(:,5) - b * chassis_x(:,6), chassis_x(:,4)) - chassis_u(:,3);
    rvx = chassis_x(:,4) .* chassis_x(:,6) + chassis_dx(:,5);

    subplot(3,3,8)
    grid on;
    hold on;
    plot(alpha_f * 180 / pi, rvx, 'b')
    % plot(t, - 24 * ones(size(t)), 'k-.')
    % plot(t, 24 * ones(size(t)), 'k-.')
    set(gca, 'FontSize', scale*4)
    xlabel('Front Sideslip angle - \alpha_f [\circ]', 'FontSize', scale*8)
    ylabel('Lat. acceleration - a_y [m/s^2]', 'FontSize', scale*8)

    subplot(3,3,9)
    grid on;
    hold on;
    plot(alpha_r * 180 / pi, rvx, 'b')
    % plot(t, - 24 * ones(size(t)), 'k-.')
    % plot(t, 24 * ones(size(t)), 'k-.')
    set(gca, 'FontSize', scale*4)
    xlabel('Rear Sideslip angle - \alpha_r [\circ]', 'FontSize', scale*8)
    ylabel('Lat. acceleration - a_y [m/s^2]', 'FontSize', scale*8)
    
    fig.Units = 'centimeters';
    fig.PaperUnits = 'centimeters';
    fig.PaperPosition = scale*[0 0 textwidth textwidth];
    fig.PaperSize = scale*[textwidth textwidth];
    
    drawnow;
    print(['results/' name '.png'], '-dpng', ['-r' num2str(300 / scale)])
end