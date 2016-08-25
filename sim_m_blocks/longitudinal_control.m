function [throttle, brake, dvx_int] = longitudinal_control(vx, vx_ref, ax_ref, vx_int, controller)
    % longitudinal_control Discrete LQR control callback
    %    Inputs: vx     - Current velocity [m/s]
    %            vx_ref - Velocity reference [m/s]
    %            ax_ref - Acceleration reference [m/s^2]
    %            vx_int - Velocity error integral [m]
    %            controller - Controller gain structure
    %                controller.K      - Controller gains w.r.t vx [-]
    %                controller.P      - Closed loop cost matrix w.r.t vx [-]
    %                controller.Rbar   - Feasibility offset cost matrix w.r.t vx [-]
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
    throttle = u*(u > 0)/5;
    brake = u*(u < 0)/10;
end