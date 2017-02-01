function plot_invariant_set(controller)
    % plot_invariant_set Plot resulting invariant set
    %
    %    Inputs: controller - Controller gain structure
    %                 controller.lateral     - Lateral controller gain structure
    %                     lateral.vx             - Reference velocity vector [m/s]
    %                     lateral.vx_min         - Minimum reference velocity [m/s]
    %                     lateral.vx_max         - Maximum reference velocity [m/s]
    %                     lateral.K              - Controller gains w.r.t vx [-]
    %                     lateral.P              - Closed loop cost matrix w.r.t vx [-]
    %                     lateral.Rbar           - Feasibility offset cost matrix w.r.t vx [-]
    %                     lateral.F              - State transition matrix w.r.t vx [-]
    %                     lateral.G              - Control input matrix w.r.t vx [-]
    %                     lateral.offset         - YALMIP optimizer for invariant set
    %                     lateral.inv_sets       - Array of invariant set polytopes
    %                 controller.longitudinal - Longitudinal controller gain structure
    %                     longitudinal.K              - Controller gains w.r.t vx [-]
    %                     longitudinal.P              - Closed loop cost matrix w.r.t vx [-]
    %                     longitudinal.Rbar           - Feasibility offset cost matrix w.r.t vx [-]
    %                     longitudinal.ax_to_throttle - Ratio between acceleration and throttle pedal [-]
    %                     longitudinal.ax_to_brake    - Ratio between acceleration and brake pedal [-]
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    textwidth = 17.78;
    scale = 300 / get(0,'ScreenPixelsPerInch');

    vx_list = controller.lateral.vx;
    inv_sets = controller.lateral.inv_sets;

    % Get all external points
    points = [];
    for i = 1:length(vx_list)
        vx = vx_list(i);
        inv_set = inv_sets(i).minVRep();
        points = [points;  inv_set.V vx * ones(size(inv_set.V, 1), 1)];
    end

    % Generate boundary
    k = boundary(points);

    % Plot triangulation of surface
    fig = figure(1);
    clf;

    plot(alphaShape(points(:,1),points(:,2),points(:,3), 4));
    axis normal
    xlabel('Lateral velocity - v_y [m/s]', 'FontSize', scale*8)
    ylabel('Yaw rate - r [m/s]', 'FontSize', scale*8)
    zlabel('Longitudinal velocity - v_x [m/s]', 'FontSize', scale*8)
    view(90 ./ [4 4])

    % Save to disk
    fig.Units = 'centimeters';
    fig.PaperUnits = 'centimeters';
    fig.PaperPosition = scale*[0 0 textwidth textwidth];
    fig.PaperSize = scale*[textwidth textwidth];

    print('results/invariant_set.png', '-dpng', ['-r' num2str(300 / scale)])

end