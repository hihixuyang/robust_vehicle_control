function [A, Bu, Br] = linear_bicycle_model(vx, param)
    % linear_bicycle_model Linear bicycle model matrices
    %    Inputs: vx    - Vehicle longitudinal velocity [m/s]
    %            param - Parameter structure
    %                param.m  - Vehicle mass [Kg]
    %                param.l  - Wheelbase [m]
    %                param.a  - Distance from cg to front axle (x) [m]
    %                param.Iz - Yaw moment of inertia [Kgm^2/rad]
    %                param.dy - Distance from CG to crosstrack reference point [m]
    %                param.front_tire
    %                    front_tire.C  - Cornering stiffness [N/rad]
    %                param.rear_tire
    %                    rear_tire.C  - Cornering stiffness [N/rad]
    %
    %    Outputs: A - System matrix
    %             Bu - Contol input matrix
    %             Br - reference input matrix
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    % Load parameter for brevity
    m = param.body.m;
    l = param.body.l;
    a = param.body.a;
    b = l - a;
    Iz = param.body.Iz;
    dy = param.body.dy;
    Cf = param.front_tire.C;
    Cr = param.rear_tire.C;

    delta_tc = param.steering.tc;

    % Linear bicycle model
    % State: [ey, epsi, vy, r]
    % Input: [delta_f]
    % Reference: [kappa]
    % Assumptions:
    %    sin(x) ~ x (5% error at 31.6 deg)
    %    cos(x) ~ 1 (5% error at 18.2 deg)
    %    tan(x) ~ x (5% error at 21.5 deg)
    %    diff(vx) ~ 0
    Ab = zeros(2,2);
    Ab(1,1) = - (Cf + Cr) / vx / m;
    Ab(1,2) = - (a * Cf - b * Cr) / vx / m - vx;
    Ab(2,1) = - (a * Cf - b * Cr) / vx / Iz;
    Ab(2,2) = - (a^2 * Cf + b^2 * Cr) / vx / Iz;

    Bb = zeros(2,1);
    Bb(1,1) = Cf / m;
    Bb(2,1) = a * Cf / Iz;

    % Trajectory-referenced bicycle model
    % State: [ey, epsi, vy, r, delta_f]
    % Input: [delta_f_cmd]
    % Reference: [kappa]
    % Assumptions:
    %    Along-track distance (s) is not modeled

%     A = [0,      vx, 1, dy,         0;
%          0,       0, 0,  1,         0;
%          zeros(2,2),    Ab,        Bb;
%          0,       0, 0,  0, -delta_tc];
    A = [0,      vx, 1, dy;
         0,       0, 0,  1;
         zeros(2,2),    Ab];

%     Bu = [zeros(4,1);
%             delta_tc];
    Bu = [zeros(2,1);
                  Bb];

%     Br = [         0;
%                  -vx;
%           zeros(3,1)];
    Br = [         0;
                 -vx;
          zeros(2,1)];
end