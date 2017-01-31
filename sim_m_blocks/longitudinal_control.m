function [throttle, brake, dvx_int] = longitudinal_control(vx, vx_ref, ax_ref, vx_int, gear, controller)
    % longitudinal_control Discrete LQR control callback
    %    Inputs: vx     - Current velocity [m/s]
    %            vx_ref - Velocity reference [m/s]
    %            ax_ref - Acceleration reference [m/s^2]
    %            vx_int - Velocity error integral [m]
    %            controller - Controller gain structure
    %                controller.K      - Controller gains w.r.t vx [-]
    %                controller.P      - Closed loop cost matrix w.r.t vx [-]
    %                controller.Rbar   - Feasibility offset cost matrix w.r.t vx [-]
    %                controller.ax_to_throttle - Ratio between acceleration and throttle pedal [-]
    %                controller.ax_to_brake    - Ratio between acceleration and brake pedal [-]
    %
    %    Outputs: throttle - Throttle command[rad]
    %             brake    - Brake command [rad]
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    % Create controller state vector
    x_ctrl = [vx_int; vx_ref - vx; ax_ref];
    dvx_int = vx_ref - vx;

    % Calculate control input
    u = - controller.K * x_ctrl;

    % Convert to throttle and brake values
    throttle = controller.ax_to_throttle(gear+2) * u * (u >= 0);
    brake = - controller.ax_to_brake * u * (u < 0);
end