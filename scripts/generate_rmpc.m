function [controller] = generate_rmpc(vehicle_parameters, controller)
    % generate_gcmpc Generate one step RMPC
    %    Inputs: vehicle_parameters - Vehicle parameter structure
    %                vehicle_parameters.steering.angle_limit - Maximum steering angle [rad]
    %             controller - Controller gain structure
    %                 controller.lateral     - Lateral controller gain structure
    %                     lateral.vx           - Reference velocity vector [m/s]
    %                     lateral.vx_min       - Minimum reference velocity [m/s]
    %                     lateral.vx_max       - Maximum reference velocity [m/s]
    %                     lateral.K            - Controller gains w.r.t vx [-]
    %                     lateral.P            - Closed loop cost matrix w.r.t vx [-]
    %                     lateral.Rbar         - Feasibility offset cost matrix w.r.t vx [-]
    %                     lateral.F            - State transition matrix w.r.t vx [-]
    %                     lateral.G            - Control input matrix w.r.t vx [-]
    %                     lateral.offset_opts  - Array of YALMIP optimizer for invariant set
    %                     lateral.inv_sets     - Array of invariant set polytopes
    %                 controller.longitudinal - Longitudinal controller gain structure
    %                     longitudinal.K              - Controller gains w.r.t vx [-]
    %                     longitudinal.P              - Closed loop cost matrix w.r.t vx [-]
    %                     longitudinal.Rbar           - Feasibility offset cost matrix w.r.t vx [-]
    %                     longitudinal.ax_to_throttle - Ratio between acceleration and throttle pedal [-]
    %                     longitudinal.ax_to_brake    - Ratio between acceleration and brake pedal [-]
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo
    
    % Get velocity list
    ctrl = controller.lateral;
    vx_list = ctrl.vx;
    
    % Get maximum steering angle
    u_max = vehicle_parameters.steering.angle_limit;
    
    % Get system dimensions
    n_x = size(ctrl.G, 2);
    n_u = size(ctrl.G, 3);
    n_sys = size(ctrl.G, 4);
    
    % Creat optimizer problems for invariant set
    offset_opts = {};
    for i = 1:length(vx_list)
        vx = vx_list(i);
        disp(['        Iteration ' num2str(i, '%02.0f') ', vx = ' num2str(vx)])

        % Construct YALMIP optimization problem based on this cone
        x = sdpvar(n_x,1);         % State vector [vy, r]
        u = sdpvar(n_u,1);         % Controller input [delta_f]
        v = sdpvar(n_u,1);         % Feasibility offset vector [delta_f]
        f = ctrl.F(i,:,:,:);       % State transition matrix vertexes
        g = ctrl.G(i,:,:,:);       % Control input matrix vertexes
        r_bar = ctrl.Rbar(i,:,:);  % v cost hessian
        
        f = permute(f, [2,3,4,1]);
        g = permute(g, [2,3,4,1]);
        
        % Get invariant set
        inv_set = controller.lateral.inv_sets(i);

        % Create constraint set
        constraints = (-u_max <= u + v <= u_max);
        for j = 1:n_sys
            constraints = [constraints;
                           inv_set.A * (f(:,:,j) * x + g(:,:,j) * (u + v)) <= inv_set.b];
        end

        % Define optimization problem
        options = sdpsettings('solver', '+gurobi', 'verbose', 0);
        
        offset_opts{i,1} = optimizer(constraints, v' * r_bar * v, options, [x; u], v);
    end
    
    % Save optimizations
    controller.lateral.offset_opts = offset_opts;
end