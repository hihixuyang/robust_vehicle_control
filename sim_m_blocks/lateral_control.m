function [u, opt_in] = lateral_control(x, kappa, controller)
    % lateral_control Guaranteed cost control callback
    %    Inputs: x     - State vector (s, ey, epsi, vx, vy, r)
    %            kappa - Path curvature
    %            controller - Controller gain structure
    %                controller.vx     - Reference velocity vector [m/s]
    %                controller.vx_min - Minimum reference velocity [m/s]
    %                controller.vx_max - Maximum reference velocity [m/s]
    %                controller.K      - Controller gains w.r.t vx [-]
    %                controller.P      - Closed loop cost matrix w.r.t vx [-]
    %                controller.Rbar   - Feasibility offset cost matrix w.r.t vx [-]
    %
    %    Outputs: u      - Control Input [rad]
    %             opt_in - Inputs to YALMIP optimizer [-]
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    % Get relevant states
    ey = x(2);    % Crosstrack error
    epsi = x(3);  % Heading error
    vx = x(4);    % Longitudinal velocity
    vy = x(5);    % Lateral velocity
    r = x(6);     % Yaw rate

    % Create controller state vector
    x_ctrl = [ey; epsi; vy; r; kappa];

    % Gain schedule controller gain w.r.t longitudinal velocity
    K = permute(interp1(controller.vx, controller.K, ...
                        max(min(vx, controller.vx_max), controller.vx_min), 'spline'), [2 3 1]);

    % Calculate control input
    u = - K * x_ctrl;
    opt_in = [vy; r; u; vx];

end