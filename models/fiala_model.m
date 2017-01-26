function Fy = fiala_model(slip, Fz, param)
    % fiala_model Fiala tire model for lateral tire forces
    %    Inputs: slip  - Sideslip angle [rad]
    %            Fz    - Normal tire force
    %            param - Parameter structure
    %                param.C  - Tire cornering stiffness [N/rad]
    %                param.Ru - Radio between frictions [-]
    %                param.mu - Tire friction [-]
    %
    %    Outputs: Fy - Lateral tire force [N]
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo
    
    % Load parameters
    C = param.C;
    Ru = param.Ru;
    mu = param.mu;
    
    % Compensate for polynomial peak decrease when Ru < 1
    mu = mu/abs((Ru/3 - 2/3)/((2*Ru)/3 - 1)^2 - ...
                1/((2*Ru)/3 - 1) + ((2*Ru)/27 - 1/9)/((2*Ru)/3 - 1)^3);
    
    Fz = mu * Fz;
    slip_max = atan2(3 * Fz, C);

    % Calculate force in the nominal region
    Fy = - C * tan(slip) + ...
           C ^ 2 ./ (3 * mu * Fz) * (2 - Ru) .* tan(slip) .* abs(tan(slip)) - ...
           C ^ 3 ./ (3 * mu * Fz) .^ 2 * (1 - 2 * Ru / 3) .* tan(slip) .^ 3;
    % Check saturation
    Fy = Fy .* (abs(slip) < slip_max) - sign(slip) .* mu * Fz * Ru .* (abs(slip) >= slip_max);
    
end