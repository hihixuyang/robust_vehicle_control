function controller = generate_lateral_controller(vehicle_parameters, controller_parameters)
    % generate_lateral_controller Generates the GCC controller gains
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
    %
    %    Outputs: controller - Controller gain structure
    %                 controller.vx     - Reference velocity vector [m/s]
    %                 controller.vx_min - Minimum reference velocity [m/s]
    %                 controller.vx_max - Maximum reference velocity [m/s]
    %                 controller.K      - Controller gains w.r.t vx [-]
    %                 controller.P      - Closed loop cost matrix w.r.t vx [-]
    %                 controller.Rbar   - Feasibility offset cost matrix w.r.t vx [-]
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo
    
    % Cost basis definition
    tau = controller_parameters.tau;
    Qp = controller_parameters.C;
    Rp = controller_parameters.D;
    W = controller_parameters.W;

    % Gain scheduling velocity range
    vx_min = 2;
    vx_max = 40;
    vx_list = (vx_min:vx_max)';

    % Query for matrix sizes
    [A, Bu, Br, ~] = linear_bicycle_model_uncertain(1, [0.9, 0.9], vehicle_parameters);

    Nx = size(A, 2);
    Nu = size(Bu, 2);
    Nr = size(Br, 2);

    % Controller list on speed
    % K has both feedback and feedforward terms
    K_list    = zeros(size(vx_list, 1), Nu, Nx + Nr);
    P_list    = zeros(size(vx_list, 1), Nx, Nx);
    Rbar_list = zeros(size(vx_list, 1), Nu, Nu);

    % Resulting controller will be gain scheduled in speed
    for i = 1:size(vx_list, 1)
        vx = vx_list(i);
        display(['    Iteration ' num2str(i, '%02.0f') ', vx = ' num2str(vx)])

        % Get continous time matrices in affine form
        [A, Bu, Br, Hc, Ea, Ebu, ~] = ...
            linear_bicycle_model_uncertain(vx, [1, 1], vehicle_parameters);

        % Discretize uncertain system
        [F, G, H, Ef, Eg] = lct.uc2d(A, Bu, Hc, Ea, Ebu, controller_parameters.Ts);

        % Discretize reference matrix
        [~, ~, Gr, ~, ~] = lct.uc2d(A, Bu, Br, Ea, Ebu, controller_parameters.Ts);

        % Initialize exponential convergent cost
        sQp = size(Qp,1);
        Rv = zeros(sQp * size(tau,1), Nu);
        Qv = zeros(sQp * size(tau,1), Nx);

        % Create exponential convergent basis
        for j = 1:size(tau,1)
            Rv(sQp * (j - 1) + (1:sQp), :) = Qp * A^(j-1) * Bu + Rp;
            Qv(sQp * (j - 1) + (1:sQp), :) = Qp * A^(j-1) * (eye(Nx) / tau(j) + A);
        end

        % Create matrices Q and R
        Q = Qv' * W(vx) * Qv;
        R = Rv' * W(vx) * Rv;

        % Normalize them for better numerical stability
        Q = Q / trace(R);
        R = R / trace(R);

        % Generate Guaranteed Cost Control gain
        [K, P, e] = lct.gcc(F, G, 1000*H, Ef/1000, Eg/1000, Q, R);

        % Calculate zero DC gain feed-forward term w.r.t crosstrack error
        Ftilda = F - G * K;
        
        [Flist, Glist] = lct.pu2su(Ftilda, G, H, Ef, Eg);
        N = lct.rdcgain(Flist, Glist, Gr, Qp(1,:));
        K = [K, N];

        % Save related quantities for future gain scheduling
        X = inv(inv(P) - e * (H * H'));
        K_list(i, :, :) = K;
        P_list(i, :, :) = P;
        Rbar_list(i, :, :) = R + e^-1 * (Ebu' * Ebu) + G' * X * G;
    end

    % Create controller structure
    controller = struct();

    % Velocity information for gain scheduling
    controller.vx = vx_list;
    controller.vx_min = vx_min;
    controller.vx_max = vx_max;

    controller.K = K_list;        % Controller gain matrix
    controller.P = P_list;        % Closed loop state cost matrix
    controller.Rbar = Rbar_list;  % Feasbility offset cost matrix
end