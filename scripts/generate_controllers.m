function controller = generate_controllers(vehicle_parameters, controller_parameters)
    % generate_controllers Generates the lateral GCC gains and longitudinal DLQR gains
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
    %
    %    Outputs: controller - Controller gain structure
    %                 controller.lateral     - Lateral controller gain structure
    %                     lateral.vx             - Reference velocity vector [m/s]
    %                     lateral.vx_min         - Minimum reference velocity [m/s]
    %                     lateral.vx_max         - Maximum reference velocity [m/s]
    %                     lateral.K              - Controller gains w.r.t vx [-]
    %                     lateral.P              - Closed loop cost matrix w.r.t vx [-]
    %                     lateral.Rbar           - Feasibility offset cost matrix w.r.t vx [-]
    %                 controller.longitudinal - Longitudinal controller gain structure
    %                     longitudinal.K              - Controller gains w.r.t vx [-]
    %                     longitudinal.P              - Closed loop cost matrix w.r.t vx [-]
    %                     longitudinal.Rbar           - Feasibility offset cost matrix w.r.t vx [-]
    %                     longitudinal.ax_to_throttle - Ratio between acceleration and throttle pedal [-]
    %                     longitudinal.ax_to_brake    - Ratio between acceleration and brake pedal [-]
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    %
    % Lateral Controller
    %
    disp(['    Generating lateral controller'])

    % Cost basis definition
    tau = controller_parameters.tau;
    Qp = controller_parameters.C;
    Rp = controller_parameters.D;
    W = controller_parameters.W;

    % Gain scheduling velocity range
    vx_min = 1;
    vx_max = 35;
    vx_list = (vx_min:vx_max)';

    % Query for matrix sizes
    [A, Bu, Br, ~] = linear_bicycle_model_uncertain(1, vehicle_parameters);

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
        disp(['        Iteration ' num2str(i, '%02.0f') ', vx = ' num2str(vx)])

        % Get continous time matrices in affine form
        [A, Bu, Br, Hc, Ea, Ebu, ~] = ...
            linear_bicycle_model_uncertain(vx, vehicle_parameters);

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
        H  = 1000 * H;
        Ef = Ef / 1000;
        Eg = Eg / 1000;
        [K, P, e] = lct.gcc(F, G, H, Ef, Eg, Q, R);

        % Calculate zero DC gain feed-forward term w.r.t crosstrack error
        Ftilda = F - G * K;

        M1 = (Qp(1,:) / (eye(Nx) - F + G * K)) * Gr;
        M2 = (Qp(1,:) / (eye(Nx) - F + G * K)) * G;
        N = M2 \ M1;
        K = [K, N];

        % Save related quantities for future gain scheduling
        X = inv(inv(P) - (H * e * H'));
        K_list(i, :, :) = K;
        P_list(i, :, :) = P;
        Rbar_list(i, :, :) = R + Eg' * inv(e) * Eg + G' * X * G;
    end

    % Create controller structure
    controller = struct();
    controller.lateral = struct();


    % Velocity information for gain scheduling lateral controller
    controller.lateral.vx = vx_list;
    controller.lateral.vx_min = vx_min;
    controller.lateral.vx_max = vx_max;

    % Lateral controller gains
    controller.lateral.K = K_list;        % Controller gain matrix
    controller.lateral.P = P_list;        % Closed loop state cost matrix
    controller.lateral.Rbar = Rbar_list;  % Feasbility offset cost matrix

    %
    % Longitudinal controller Controller
    %
    disp(['    Generating longitudinal controller'])

    % Define system dynamics
    % State x = [ev_int; ev]
    % Input u = ax
    % Input r = ax_ref
    % Dynamics: dx = A x + B u + Br r
    A = [0 1;
         0 0];
    B = [0; -1];
    Br = [0; 1];

    % Discretize dynamics
    out = expm([A B Br; zeros(2,4)]);
    F = out(1:2, 1:2);
    G = out(1:2, 3);
    Gr = out(1:2, 4);

    % Calculate discrete LQR
    Q = eye(2);
    R = 1;
    [K, P] = dlqr(F, G, Q, R);

    % Calculate feedforward
    N = G \ Gr;
    K = [K, N];

    % Get throttle and brake constants
    [ax_to_throttle, ax_to_brake] = longitudinal_model(vehicle_parameters);

    % Create controller structure
    controller.longitudinal = struct();
    controller.longitudinal.K = K;
    controller.longitudinal.P = P;
    controller.longitudinal.Rbar = R + G' * P * G;
    controller.longitudinal.ax_to_throttle = ax_to_throttle;
    controller.longitudinal.ax_to_brake = ax_to_brake;

end