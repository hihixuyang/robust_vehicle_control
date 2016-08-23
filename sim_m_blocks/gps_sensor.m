function [ y ] = gps_sensor( x, simulation_parameters )

    mu = simulation_parameters.gps.C * x;
    sigma = simulation_parameters.gps.Cov;

    % Multivariate normal distribution
    disturb = chol(sigma) * randn(length(mu),1);
    
    % Rotate velocity vector by heading disturbance
    cpsi = cos(disturb(2));
    spsi = sin(disturb(2));
    R = [cpsi, -spsi; spsi cpsi];
    mu(3:4) = R * mu(3:4);
    
    % Add disturbance
    y = mu + disturb;

end

