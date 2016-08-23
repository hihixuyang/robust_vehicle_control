function [ Fy_max, slip_max ] = peak_fiala_model(Fz, mu, param)
    %peak_fiala_model Calculates the maximum tire force and its sideslip angle
    %    Inputs: Fz    - Normal tire force
    %            mu    - Tire friction
    %            param - Parameter structure
    %                param.C - Tire cornering stiffness [N/rad]
    %                param.Ru - Radio between frictions [-]
    %
    %    Outputs: Fy - Lateral tire force [N]
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    % Load parameters
    C = param.C;
    Ru = param.Ru;

    % Calcualte Fy_max
    Fy_max = mu * Fz;

    q = (1 - 2 * Ru / 3)^(-1);
    slip_max = atan2(q * Fy_max, C);
end

