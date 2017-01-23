function [trajectory] = load_trajectory(filename)
    % load_trajectory Loads trajectory information from csv file
    %    Inputs: filename - Filename for trajectory to be loaded
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
    trajectory.s = X(1:end,1);
    trajectory.kappa = X(1:end,2);
    trajectory.vx = X(1:end,3);
    trajectory.ax = X(1:end,4);
end

