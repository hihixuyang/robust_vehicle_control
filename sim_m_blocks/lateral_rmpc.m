function [ v ] = lateral_rmpc(u, controller)
    % lateral_observer Guaranteed cost observer callback
    %    Inputs: u          - Optimization input (ey, epsi, delta_ref, vx)
    %            controller - Controller gain structure
    %                controller.vx     - Reference velocity vector [m/s]
    %                controller.offset_opts - Array of YALMIP optimizers
    %
    %    Outputs: v - Steering offset for feasibility
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    % Get longitduinal velocity and remove it from inputs
    vx = u(end);
    u = u(1:end-1);

    % Find best optimizer to use
    [~, idx] = min(controller.vx - round(vx));

    % Execute it and return solution
    solver = controller.offset_opts{idx};
    v = solver(u);

end