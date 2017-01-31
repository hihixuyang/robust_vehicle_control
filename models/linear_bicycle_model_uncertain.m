function [ A, Bu, Br, H, Ea, Ebu, Ebr, slip_max] = linear_bicycle_model_uncertain(vx, param)
    % linear_bicycle_model_uncertain Generates the linear bicycle model matrices
    %                                with tire parameter uncertainty
    %    Inputs: vx    - Vehicle longitudinal velocity [m/s]
    %            param - Parameter structure
    %                param.m  - Vehicle mass [Kg]
    %                param.l  - Wheelbase [m]
    %                param.a  - Distance from cg to front axle (x) [m]
    %                param.h  - Distance from cg to ground (z) [m]
    %                param.Iz - Yaw moment of inertia [Kgm^2/rad]
    %                param.g  - Gravity acceleration [m/s^2]
    %                param.front_tire
    %                    front_tire.C  - Cornering stiffness [N/rad]
    %                    front_tire.Ru - Radio between frictions [-]
    %                    front_tire.mu - Road/tire static friction [-]
    %                param.rear_tire
    %                    rear_tire.C  - Cornering stiffness [N/rad]
    %                    rear_tire.Ru - Radio between frictions [-]
    %                    rear_tire.mu - Road/tire static friction [-]
    %
    %    Outputs: A        - System matrix
    %             Bu       - Contol input matrix
    %             Br       - Reference input matrix
    %             H        - Disturbance input matrix
    %             Ea       - State disturbance matrix
    %             Ebu      - Control input disturbance matrix
    %             Ebr      - Reference input disturbance matrix
    %             slip_max - Maximum robust slip angles
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    % Load parameter for brevity
    m = param.body.m;
    l = param.body.l;
    a = param.body.a;
    b = l - a;
    dy = param.body.dy;

    g = param.world.g;

    C_f = param.front_tire.C;
    Ru_f = param.front_tire.Ru;
    mu_f = param.front_tire.mu;

    C_r = param.rear_tire.C;
    Ru_r = param.rear_tire.Ru;
    mu_r = param.rear_tire.mu;

    % Tire parameters
    Fz_f = m * (b * g) / l;
    Fz_r = m * (a * g) / l;

    % Maximum cornering stiffness is trivial
    C_f_max = C_f + param.uncertainty.C_f;
    C_r_max = C_r + param.uncertainty.C_r;

    % Minimum cornering stiffness depends on tire saturation
    C_f_min = Inf;
    C_r_min = Inf;
    slip_f_max = Inf;
    slip_r_max = Inf;
    for i = 0:7
        sign_C = 2 * mod(floor(i / 4), 2) - 1;
        sign_Ru = 2 * mod(floor(i / 2), 2) - 1;
        sign_mu = 2 * mod(i, 2) - 1;

        % Front tire
        tire_param = struct();
        tire_param.C = C_f + sign_C * param.uncertainty.C_f;
        tire_param.Ru = Ru_f + sign_Ru * param.uncertainty.Ru_f;
        tire_param.mu = mu_f + sign_mu * param.uncertainty.mu_f;
        [Fy_max, slip_max] = peak_fiala_model(Fz_f, tire_param.mu, tire_param);
        C_f_min = min(C_f_min, Fy_max / slip_max);
        slip_f_max = min(slip_f_max, slip_max);

        % Rear tire
        tire_param = struct();
        tire_param.C = C_r + sign_C * param.uncertainty.C_r;
        tire_param.Ru = Ru_r + sign_Ru * param.uncertainty.Ru_r;
        tire_param.mu = mu_r + sign_mu * param.uncertainty.mu_r;
        [Fy_max, slip_max] = peak_fiala_model(Fz_r, tire_param.mu, tire_param);
        C_r_min = min(C_r_min, Fy_max / slip_max);
        slip_r_max = min(slip_r_max, slip_max);
    end

    % Calculate the intermediary plant
    param.front_tire.C = (C_f_max + C_f_min) / 2;
    param.rear_tire.C = (C_r_max + C_r_min) / 2;
    [A, Bu, Br] = linear_bicycle_model(vx, param);

    % Calculate the uncertainty matrices
    dC_f = (C_f_max - C_f_min) / 2;
    dC_r = (C_r_max - C_r_min) / 2;
    [H, Ea, Ebu] = cornering_stiffness_uncertainty(vx, [dC_f, dC_r], param);

    H = [zeros(2); H];
    Ea = [zeros(2) Ea];
    Ebu = Ebu;
    Ebr = zeros(2, 1);

    slip_max = [slip_f_max, slip_r_max];
end