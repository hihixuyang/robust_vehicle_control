function plot_lateral_controller(vehicle_parameters, controller_parameters, controller)
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
        [A, Bu, Br, Hc, Ea, Ebu, ~] = ...
            linear_bicycle_model_uncertain(vx, vehicle_parameters);

        % Discretize uncertain system
        [F, G, H, Ef, Eg] = lct.uc2d(A, Bu, Hc, Ea, Ebu, controller_parameters.Ts);

        % Discretize reference matrix
        [~, ~, Gr, ~, ~] = lct.uc2d(A, Bu, Br, Ea, Ebu, controller_parameters.Ts);

        % Define uncertain variables
        nCf = ureal('nCf', 0, 'range', [-1, 1]);
        nCr = ureal('nCf', 0, 'range', [-1, 1]);
        delta = diag([nCf, nCr]);

        [~, Kidx] = max(controller.lateral.vx == vx);
        K = permute(controller.lateral.K(Kidx, :, 1:size(A,2)), [2, 3, 1]);
        N = permute(controller.lateral.K(Kidx, :, size(A,2)+1:end), [2, 3, 1]);

        % Open loop plant
        Fu = F + H * delta * Ef;
        Gu = G + H * delta * Eg;
        Pol = ss(Fu, Gu, eye(size(A)), 0, controller_parameters.Ts);

        % Plot loop sentivity
        loops = loopsens(Pol,K);

        subplot(2,3,i)
        sigma(loops.Li,'g',loops.Ti,'r',loops.Si,'b', 10.^[-2:0.01:3])
        hold on
        grid on
        ylim([-50, 50])
        set(gca,'xtick',10.^([-2:3]))
        xlabel('Frequency [rad]', 'FontSize', scale*8)
        ylabel('Singular values [dB]', 'FontSize', scale*8)
        title(['Singular values for vx = ' num2str(vx) ' m/s'], 'FontSize', scale*8)
        legend('Li', 'Ti', 'Si')

        % Closed loop plant
        Pcl = ss(Fu - Gu * K, Gr - Gu * N, ...
                 eye(1, size(A, 2)), 0, controller_parameters.Ts);

        % Curvature reference to crosstrack error plant plot
        subplot(2,3,i+3)
        hold on
        sigma(Pcl, 10.^[-2:0.01:3])
        grid on
        set(gca,'xtick',10.^([-2:3]))
        xlabel('Frequency [rad]', 'FontSize', scale*8)
        ylabel('Singular values [dB]', 'FontSize', scale*8)
        title(['\kappa(z) / e_y(z) for vx = ' num2str(vx) ' m/s'], 'FontSize', scale*8)
        set(gca, 'FontSize', scale*4)
    end

    fig.Units = 'centimeters';
    fig.PaperUnits = 'centimeters';
    fig.PaperPosition = scale*[0 0 textwidth 2/3 * textwidth];
    fig.PaperSize = scale*[textwidth 2/3 * textwidth];

    print('results/controller_tf.png', '-dpng', ['-r' num2str(300 / scale)])
end