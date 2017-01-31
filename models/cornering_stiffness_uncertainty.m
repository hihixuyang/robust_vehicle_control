function [H, Ea, Eb] = cornering_stiffness_uncertainty(vx, dC, param)
    % linear_bicycle_model_uncertainty Tire uncertainty matrices for linear
    % bicycle model
    %    Inputs: vx    - Vehicle longitudinal velocity [m/s]
    %            dC    - Amplitude of cornerring stiffness uncertainty
    %                    (Cmax - Cmin) / 2
    %            param - Parameter structure
    %                param.m  - Vehicle mass [Kg]
    %                param.l  - Wheelbase [m]
    %                param.a  - Distance from cg to front axle (x) [m]
    %                param.Iz - Yaw moment of inertia [Kgm^2/rad]
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

    % Cornering stiffness uncertainty
    dCf = dC(1);
    dCr = dC(2);

    In   = diag([1/m, 1/Iz]);
    LArm = [1,  1;
            a, -b];
    DMag = diag([dCf, dCr]);
    S    = [1; 0];

    % 1000 factor to improve numerical stability
    H  = In * LArm;
    Ea = - DMag * LArm' / vx;
    Eb = DMag * S;
end