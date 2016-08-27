function [trajectory] = load_trajectory(filename, ay_max)
    % load_trajectory Loads trajectory information from csv file
    %    Inputs: filename - Filename for trajectory to be loaded
    %            ay_max   - Maximum lateral acceleration for speed profile
    %
    %    Outputs: trajectory - Trajectory structure
    %                 trajectory.s     - Alongtrack distance reference
    %                 trajectory.kappa - Curvature reference
    %                 trajectory.vx    - Longitudinal velocity reference
    %                 trajectory.ax    - Longitudinal acceleration reference
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo
    
    X = csvread(['trajectories/' filename]);
    
    % Create trajectory structure
    trajectory = struct();
    
    % Load curvature profile
    trajectory.s = X(:,1);
    trajectory.kappa = X(:,2);
    
    % Calculate maximum constant velocity profile
    vx_max = min(sqrt(ay_max ./ X(:,2)));
    
    % Set constant velocity trajectory
    trajectory.vx = vx_max * ones(size(X(:,2)));
    trajectory.ax = zeros(size(X(:,2)));
end

