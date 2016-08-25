function [ ax_to_throttle, ax_to_brake ] = longitudinal_model(vehicle_parameters)
    % longitudinal_model Calculates the ratio between acceleration and pedal positions
    %    Inputs: vehicle_parameters - Parameter structure
    %                vehicle_parameters.powertrain - Powertrain parameter structure
    %                    powertrain.wheel_radius      - Road wheel radius [m]
    %                    powertrain.gear_ratio        - Gear ratio for all possible gears [-]
    %                    powertrain.diff_ratio        - Diferential ratio [-];
    %                    powertrain.efficiency        - Overall powetrain efficiency [-]
    %                    powertrain.engine_rpm_map    - Torque map's engine speed [rpm]
    %                    powertrain.engine_torque_map - Torque map's torque output [Nm]
    %                vehicle_parameters.brake      - Brake parameter structure
    %                    brake.wheel_radius          - Road wheel radius [m]
    %                    brake.mc_pressure_to_torque - Ratio between mc pressure and calyper torque [Nm/Pa]
    %                    brake.pedal_to_mc_pressure  - Ratio between pedal travel and mc pressure[Pa]
    %                vehicle_parameters.body       - Body parameter structure
    %                    body.m - Vehicle mass [Kg]
    %
    %    Outputs: ax_to_throttle - Ratio between acceleration and throttle
    %             ax_to_brake    - Ratio between acceleration and brake
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo
    
    pwt_params = vehicle_parameters.powertrain;
    brk_params = vehicle_parameters.brake;
    
    % Get powertrain parameters
    wheel_radius = pwt_params.wheel_radius;
    diff_ratio = pwt_params.diff_ratio;
    efficiency = pwt_params.efficiency;
    mass = vehicle_parameters.body.m;
    
    % Initialize vector of ratios
    ax_to_throttle = zeros(size(pwt_params.gear_ratio));
    
    % Calculate mean open throttle engine torque between 1000 rpm and 4000 rpm
    mask = (pwt_params.engine_rpm_map >= 1000) & (pwt_params.engine_rpm_map <= 4000);
    mean_engine_torque = mean(pwt_params.engine_torque_map(mask));

    % Calculate the steady-state ratio between acceleration and throttle position
    for i = 1:length(pwt_params.gear_ratio)
        gear_ratio = pwt_params.gear_ratio(i);
        
        ax_to_torque = wheel_radius * mass / (gear_ratio * diff_ratio * efficiency);
        torque_to_throttle = 1 / mean_engine_torque;
        
        ax_to_throttle(i) = ax_to_torque * torque_to_throttle;
    end
    
    % Set neutral ratio to 1st gear
    ax_to_throttle(2) = ax_to_throttle(3);
    
    % Get brake parameters
    wheel_radius = brk_params.wheel_radius;
    mc_pressure_to_torque = brk_params.mc_pressure_to_torque;
    pedal_to_mc_pressure = brk_params.pedal_to_mc_pressure;
    
    % Calculate the steady-state ratio between acceleration and brake position
    ax_to_brake = wheel_radius * mass / (pedal_to_mc_pressure * mc_pressure_to_torque);

end