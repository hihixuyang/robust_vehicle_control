function [controller] = generate_invariant_set(vehicle_parameters, controller_parameters, controller)
    % generate_invariant_set Generates an invariant set and the related QP optimization
    %
    %    Inputs: vehicle_parameters - Vehicle parameter structure
    %                vehicle_parameters.m  - Vehicle mass [Kg]
    %                vehicle_parameters.l  - Wheelbase [m]
    %                vehicle_parameters.a  - Distance from cg to front axle (x) [m]
    %                vehicle_parameters.h  - Distance from cg to ground (z) [m]
    %                vehicle_parameters.Iz - Yaw moment of inertia [Kgm^2/rad]
    %                vehicle_parameters.g  - Gravity acceleration [m/s^2]
    %                    front_tire.C  - Cornering stiffness [N/rad]
    %                    front_tire.Ru - Radio between frictions [-]
    %                    front_tire.mu - Road/tire static friction [-]
    %                param.rear_tire
    %                    rear_tire.C  - Cornering stiffness [N/rad]
    %                    rear_tire.Ru - Radio between frictions [-]
    %                    rear_tire.mu - Road/tire static friction [-]
    %            controller_parameters - Controller parameter structure
    %                controller_parameters.Ts - Controller sampling time
    %                controller_parameters.tau - Target time constants
    %                controller_parameters.C - State-to-output matrix
    %                controller_parameters.D - Input-to-output matrix
    %                controller_parameters.W - Output cost matrix
    %            controller - Controller gain structure
    %                controller.lateral     - Lateral controller gain structure
    %                    lateral.vx_min         - Minimum reference velocity [m/s]
    %                    lateral.vx_max         - Maximum reference velocity [m/s]
    %                    lateral.Rbar           - Feasibility offset cost matrix w.r.t vx [-]
    %
    %    Outputs: controller - Controller gain structure
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

    % Invariant cone set V-rep points
    inv_sets = Polyhedron([]);
    
    % Get velocity list
    vx_list = controller.lateral.vx;
    
    % Fetch vehicle parameters
    param = vehicle_parameters;
    
    % Get system size
    [~, Bu, ~, Hc] = linear_bicycle_model_uncertain(1, param);
    n_x = 2;
    n_u = size(Bu,2);
    n_sys = 2^size(Hc,2);

    % System and input matrices gain shcheduling variables
    F_list = zeros(length(vx_list), n_x, n_x, n_sys);
    Gu_list = zeros(length(vx_list), n_x, n_u, n_sys);

    % Calculate the invariant set for every speed a controller was designed
    for i = 1:length(vx_list)
        vx = vx_list(i);
        display(['        Iteration ' num2str(i, '%02.0f') ', vx = ' num2str(vx)])

        % Load uncertain model
        [A, Bu, ~, Hc, Ea, Ebu, ~, slip_max] = linear_bicycle_model_uncertain(vx, param);
        
        % Get system size
        n_x = size(A,2);
        n_u = size(Bu,2);
        n_w = size(Hc,2);

        % Discretize uncertain system
        [F, Gu, H, Ef, Egu] = lct.uc2d(A, Bu, Hc, Ea, Ebu, controller_parameters.Ts);

        % Get vy and r dynamics only
        mask = boolean([0;0;1;1]);
        n_x = sum(mask);

        F = F(mask, mask);
        Gu = Gu(mask,:);
        H = H(mask,:);
        Ef = Ef(:,mask);

        % Calculate list of system vertexes
        n_sys = 2 ^ n_w;
        F_vertex = zeros(n_x, n_x, n_sys);
        Gu_vertex = zeros(n_x, n_u, n_sys);
        % Gr is zero!

        % TODO: Generalize if there is more than 2 uncertainties
        for j = 1:n_sys
            deltaCf = floor((j-1)/2);
            deltaCr = ceil((j-1)/2) - deltaCf;
            Delta = 2*diag([deltaCf, deltaCr]) - eye(n_w);

            F_vertex(:,:,j) = F + H * Delta * Ef;
            Gu_vertex(:,:,j) = Gu + H * Delta * Egu;
        end
        
        F_list(i,:,:,:) = F_vertex;
        Gu_list(i,:,:,:) = Gu_vertex;

        % Define state and control input domain
        %     - a_r_max vx <= vy - b r <= a_r_max vx
        %     - a_f_max vx <= vy + a r - delta_f <= a_f_max vx
        %     - u_max <= u <= u_max
        state_mask = 1:2;  % Mask for states (discard inputs)

        u_max = param.steering.angle_limit;  % Maximum steering angle

        % Distance from CG to front (a) and rear (b) axles.
        a = param.body.a;
        b = param.body.l - param.body.a;

        % Maximum robust sideslip at front and rear axles
        slip_max_f = slip_max(1);
        slip_max_r = slip_max(2);

        % State/Control input feasible set
        x = Polyhedron([ 1,  a, -vx
                        -1, -a,  vx
                         1, -b,  0;
                        -1,  b,  0;
                         0,  0,  1;
                         0,  0, -1;],...
                       [vx * slip_max_f;
                        vx * slip_max_f;
                        vx * slip_max_r;
                        vx * slip_max_r;
                        u_max;
                        u_max]);

        inv_set = x.projection(state_mask);
        while true
            % Initialize invariant set as feasible set
            inv_set_new = x;
            % Calculate a one step set for each vertex of matrix uncertain polytope
            for j = 1:n_sys
                one_step_i = Polyhedron(inv_set.A * [F_vertex(:,:,j) Gu_vertex(:,:,j)], inv_set.b);
                inv_set_new = inv_set_new.intersect(one_step_i);
            end
            % Project to state space
            inv_set_new = inv_set_new.projection(state_mask);
            inv_set_new = inv_set_new.minHRep();

            % Check if it has converged, if not iterate
            if abs(inv_set_new.volume - inv_set.volume) < 1e-5
                break
            else
                inv_set = inv_set_new;
            end
        end

        inv_sets(i) = inv_set.minHRep();
    end

%     % Construct YALMIP optimization problem based on this cone
%     x = sdpvar(n_x,1);                    % State vector [vy, r]
%     u = sdpvar(n_u,1);                    % Controller input [delta_f]
%     v = sdpvar(n_u,1);                    % Feasibility offset vector [delta_f]
%     vx = sdpvar();                        % Current longitudinal velocity [v_x]
%     f = sdpvar(n_x, n_x, n_sys, 'full');  % State transition matrix vertexes
%     g = sdpvar(n_x, n_u, n_sys, 'full');  % Control input matrix vertexes
%     r_bar = sdpvar(n_u, n_u);             % v cost hessian
% 
%     % Create constraint set
%     constraints = [];
%     for i = 1:n_sys
%         constraints = [constraints;
%                        inv_set.A * [f(:,:,i) * x + g(:,:,i) * (u + v); vx] <= inv_set.b];
%     end
% 
%     % Define optimization problem
%     options = sdpsettings('solver', '+gurobi', 'verbose', 0);
%     solver = optimizer(constraints, v' * r_bar * v, options, [x; u; vx; r_bar; f(:); g(:)], v);
%     
%     controller.lateral.F = F_list;
%     controller.lateral.G = Gu_list;
    controller.lateral.inv_sets = inv_sets;
%     controller.lateral.offset = solver;
end