function observer = generate_lateral_observer(vehicle_parameters, observer_parameters)
    % generate_lateral_observer Generates the GCC observer gains
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
    %            observer_parameters - Observer parameter structure
    %                observer_parameters.Ts - Observer sampling time
    %                observer_parameters.rho - LTR scalar
    %                observer_parameters.subsampling - Number of timesteps between measurements
    %
    %    Outputs: observer - Observer gain structure
    %                 observer.vx          - Reference velocity vector [m/s]
    %                 observer.vx_min      - Minimum reference v`Nx + Nr);
    %                 observer.L           - Observer gains w.r.t vx [-]
    %                 observer.P           - Cost matrix w.r.t vx [-]
    %                 observer.F           - State transition matrix w.r.t vx [-]
    %                 observer.G           - Input matrix w.r.t vx [-]
    %                 observer.Gr          - Reference matrix w.r.t vx [-]
    %                 observer.C           - Output matrix w.r.t vx [-]
    %                 observer.subsampling - Number of timesteps between measurements
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    % Gain scheduling velocity range
    vx_min = 2;
    vx_max = 35;
    vx_list = (vx_min:vx_max)';

    % Observer subsampling rate
    ss = observer_parameters.subsampling;

    % Query for matrix sizes
    [A, Bu, Br, Hc] = linear_bicycle_model_uncertain(1, vehicle_parameters);

    Nx = size(A, 2);
    Nu = size(Bu, 2);
    Nr = size(Br, 2);
    Nw = size(Hc, 2);
    Ny = size(observer_parameters.C, 1);

    % Observer list on speed
    L_list    = zeros(size(vx_list, 1), Nx, Ny);
    P_list    = zeros(size(vx_list, 1), Nx, Nx);

    F_list  = zeros(size(vx_list, 1), Nx, Nx);
    G_list  = zeros(size(vx_list, 1), Nx, Nu);
    Gr_list  = zeros(size(vx_list, 1), Nx, Nr);

    % Resulting observer will be gain scheduled in speed
    for i = 1:size(vx_list, 1)
        vx = vx_list(i);
        disp(['    Iteration ' num2str(i, '%02.0f') ', vx = ' num2str(vx)])

        % Get continous time matrices in affine form
        [A, Bu, ~, Hc, Ea, Ebu, ~] = ...
            linear_bicycle_model_uncertain(vx, vehicle_parameters);

        % Discretize uncertain system
        [F, G, H, Ef, ~] = lct.uc2d(A, Bu, Hc, Ea, Ebu, observer_parameters.Ts);

        % Discretize reference matrix
        [~, ~, Gr, ~, ~] = lct.uc2d(A, Bu, Br, Ea, Ebu, observer_parameters.Ts);

        % Initialize rotation matrix
        M = eye(Nx);
        M = [M(:,end), M(:,1:end-1)];

        % Square up system
        Gbar = zeros(Nx, Nx * Nu);
        for j = 1:Nx
            Gbar(:, (j-1)*Nu+1:j*Nu) = M^(j-1) * G;
        end

        % Create matrices Q and R
        Q = Gbar * eye(Nu) * Gbar';
        R = observer_parameters.rho * eye(size(observer_parameters.C, 1));

        % Generate Guaranteed Cost Control gain
        [K, P, ~] = lct.gcc((F^ss)', observer_parameters.C', Ef'/1000, 1000*H', zeros(Nw,Ny), Q, R);
        L = K';

        % Save related quantities for future gain scheduling
        L_list(i, :, :) = L;
        P_list(i, :, :) = P;

        F_list(i, :, :) = F;
        G_list(i, :, :) = G;
        Gr_list(i, :, :) = Gr;
    end

    % Create observer structure
    observer = struct();

    % Velocity information for gain scheduling
    observer.vx = vx_list;
    observer.vx_min = vx_min;
    observer.vx_max = vx_max;

    observer.L = L_list;  % Observer gain matrix
    observer.P = P_list;  % Closed loop state cost matrix

    % System dynamics
    observer.F = F_list;
    observer.G = G_list;
    observer.Gr = Gr_list;

    % Output matrix
    observer.C = observer_parameters.C;

    % Subsampling when compared to controller
    observer.subsampling = ss;

end