function plot_lateral_observer(vehicle_parameters, controller_parameters, ...
                               observer_parameters, controller, observer)
    % plot_lateral_controller Plot input sensitivity, complementary
    %                         sentivity and open loop transfer functions
    %    Inputs: vehicle_parameters - Vehicle parameter structure
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
    %            controller_parameters - Controller parameter structure
    %                controller_parameters.Ts - Controller sampling time
    %                controller_parameters.tau - Target time constants
    %                controller_parameters.C - State-to-output matrix
    %                controller_parameters.D - Input-to-output matrix
    %                controller_parameters.W - Output cost matrix
    %            controller - Controller gain structure
    %                controller.lateral - Lateral controller gain structure
    %                    lateral.vx     - Reference velocity vector [m/s]
    %                    lateral.vx_min - Minimum reference velocity [m/s]
    %                    lateral.vx_max - Maximum reference velocity [m/s]
    %                    lateral.K      - Controller gains w.r.t vx [-]
    %                    lateral.P      - Closed loop cost matrix w.r.t vx [-]
    %                    lateral.Rbar   - Feasibility offset cost matrix w.r.t vx [-]
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    textwidth = 17.78;
    scale = 300 / get(0,'ScreenPixelsPerInch');

    vx_list = [10, 20, 30]';

    fig = figure(1);
    clf

    for i = 1 : size(vx_list, 1)
        vx = vx_list(i);
        disp(['    Iteration ' num2str(i, '%02.0f') ', vx = ' num2str(vx)])

         % Get continous time matrices in affine form
        [A, Bu, ~, Hc, Ea, Ebu, ~] = ...
            linear_bicycle_model_uncertain(vx, vehicle_parameters);

        % Discretize uncertain system
        [F, G, H, Ef, Eg] = lct.uc2d(A, Bu, Hc, Ea, Ebu, controller_parameters.Ts);

        % Define uncertain variables
        nCf = ureal('nCf', 0, 'range', [-1, 1]);
        nCr = ureal('nCf', 0, 'range', [-1, 1]);
        delta = diag([nCf, nCr]);

        [~, Kidx] = max(controller.lateral.vx == vx);
        K = permute(controller.lateral.K(Kidx, :, 1:size(A,2)), [2, 3, 1]);
        N = permute(controller.lateral.K(Kidx, :, size(A,2)+1:end), [2, 3, 1]);
        [~, Lidx] = max(observer.vx == vx);
        L = permute(observer.L(Lidx, :, :), [2, 3, 1]);

        % Open loop plant
        Fu = F + H * delta * Ef;
        Gu = G + H * delta * Eg;
        Pol = ss(Fu, Gu, eye(size(A)), 0, controller_parameters.Ts);

        % Plot loop sentivity
        ref_loops = loopsens(Pol,K);

        Pol_out = ss(Fu, Gu, observer_parameters.C, 0, controller_parameters.Ts);
        Hz = ss(F - G * K - L * observer_parameters.C, L, K, 0, controller_parameters.Ts);
        rec_loops = loopsens(Pol_out,Hz);

        subplot(1,3,i)
        sigma(ref_loops.Li,'g',ref_loops.Ti,'r',ref_loops.Si,'b', 10.^[-2:0.01:3])
        hold on
        grid on
        sigma(rec_loops.Li,'g--',rec_loops.Ti,'r--',rec_loops.Si,'b--', 10.^[-2:0.01:3])
        ylim([-50, 50])
        set(gca,'xtick',10.^([-2:3]))
        xlabel('Frequency [rad]', 'FontSize', scale*8)
        ylabel('Singular values [dB]', 'FontSize', scale*8)
        title(['Singular values for vx = ' num2str(vx) ' m/s'], 'FontSize', scale*8)
        legend('Reference Li', 'Reference Ti', 'Reference Si', ...
               'Recovered Li', 'Recovered Ti', 'Recovered Si')
        set(gca, 'FontSize', scale*4)
    end

    fig.Units = 'centimeters';
    fig.PaperUnits = 'centimeters';
    fig.PaperPosition = scale*[0 0 textwidth 1/3 * textwidth];
    fig.PaperSize = scale*[textwidth 1/3 * textwidth];

    print('results/observer_tf.png', '-dpng', ['-r' num2str(300 / scale)])
end