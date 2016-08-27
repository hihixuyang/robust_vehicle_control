function [ kappa, vx_ref, ax_ref ] = trajectory_reference(s, trajectory_params)
    % trajectory_reference Calculated trajectory reference values
    %    Inputs: s - Current along-track position
    %            trajectory_params - Trajecotry parameter structure
    %                trajectory_params.s     - Along-track distance list [m]
    %                trajectory_params.kappa - Curvature list [1/m]
    %                trajectory_params.vx    - Longitudinal velocity list [m/s]
    %                trajectory_params.ax    - Acceleration velocity list [m/s^2];
    %
    %    Outputs: kappa  - Current curvature [1/m]
    %             vx_ref - Current velocity reference [m/s]
    %             ax_ref - Current acceleration reference [m/s^2]
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo
    
    kappa = interp1(trajectory_params.s, trajectory_params.kappa, s);
    vx_ref = interp1(trajectory_params.s, trajectory_params.vx, s);
    ax_ref = interp1(trajectory_params.s, trajectory_params.ax, s);

end

