% sim_init Initialization script for simulation environment
%
%    Author: Carlos M. Massera
%    Instituition: University of São Paulo

%% Start-up
tic

% Clean up
clear all; close all; clc

display('Initializing Simulation Environment')

% Add subfolders to path
addpath('models')
addpath('scripts')
addpath('sim_m_blocks')

% Plot results flag
is_plotting_results = true;

if is_plotting_results
    display('    Warning: Intermediary results will be plotted')
end

%% Define vehicle model parameters
display('Setting vehicle parameters')

vehicle_parameters = struct();

% Body params
vehicle_parameters.body.m  = 1231;
vehicle_parameters.body.l  = 2.6;
vehicle_parameters.body.a  = 1.065;
vehicle_parameters.body.h  = 0.56;
vehicle_parameters.body.Iz = 2031.4;
% Vehicle position w.r.t rear axle
b = vehicle_parameters.body.l - vehicle_parameters.body.a;
vehicle_parameters.body.dy = - b;

% Front tire params
vehicle_parameters.front_tire.C = 120000;
vehicle_parameters.front_tire.Ru = 0.8;

% Rear tire params
vehicle_parameters.rear_tire.C = 160000;
vehicle_parameters.rear_tire.Ru = 0.8;

% Aerodynamical params
vehicle_parameters.aero.CdA = 1.49 * 1.64 * 0.3;
vehicle_parameters.aero.rho = 1.2754;

% Load transfer params
vehicle_parameters.uncertainty.load_transfer_Ts = 0.01;

% Steering system params
vehicle_parameters.steering.delay = 0.05;
vehicle_parameters.steering.tc = 1/2;
vehicle_parameters.steering.rate_limit = 19.04 * pi / 180;
vehicle_parameters.steering.angle_limit = 24 * pi / 180;

% Powertrain params
vehicle_parameters.powertrain.delay = 0.15;
vehicle_parameters.powertrain.wheel_radius = 0.33;
vehicle_parameters.powertrain.gear_shift_delay = 0.4;
vehicle_parameters.powertrain.gear_ratio = [-3.168 0 3.538 2.06 1.404 1 0.713 0.582]';
vehicle_parameters.powertrain.diff_ratio = 4.1;
vehicle_parameters.powertrain.efficiency = 0.90;
vehicle_parameters.powertrain.throttle_tc = 0.5;
vehicle_parameters.powertrain.engine_tc = 1;
vehicle_parameters.powertrain.engine_rpm_map = [0; 86.6670; 173.330; 433.330; 750; 866.670; ...
                                                1300; 1733.30; 2166.70; 2600; 3033.30; 3466.70; ...
                                                3900; 4333.30; 4766.70; 5200; 5633.30; 6066.70; ...
                                                6500];
vehicle_parameters.powertrain.engine_torque_map = [0; 88.7680; 127.540; 179.630; 194.890; ...
                                                   210.250; 229.70; 235.90; 243.460; 262.660; ...
                                                   267.30; 252.230; 254.950; 253.60; 244.640; ...
                                                   229.370; 207.420; 182.70; -61.60];

% Brake System params
vehicle_parameters.brake.delay = 0.15;
vehicle_parameters.brake.front_ratio = 0.7;
vehicle_parameters.brake.rear_ratio = 0.3;
vehicle_parameters.brake.actuator_tc = 0.1;
vehicle_parameters.brake.rate_limit = 1.4;
vehicle_parameters.brake.mc_pressure_to_torque = 620;
vehicle_parameters.brake.pedal_to_mc_pressure = 15;
vehicle_parameters.brake.hydraulic_ts = 0.15;
vehicle_parameters.brake.wheel_radius = 0.33;

% World params
vehicle_parameters.world.g = 9.81;

% Uncertainty params
vehicle_parameters.uncertainty.C_f = 0.2 * vehicle_parameters.front_tire.C;
vehicle_parameters.uncertainty.Ru_f = 0.1 * vehicle_parameters.front_tire.Ru;
vehicle_parameters.uncertainty.mu_f = 0.1;
vehicle_parameters.uncertainty.C_r = 0.2 * vehicle_parameters.rear_tire.C;
vehicle_parameters.uncertainty.Ru_r = 0.1 * vehicle_parameters.rear_tire.Ru;
vehicle_parameters.uncertainty.mu_r = 0.1;

display(['    Completed, ' num2str(toc) ' seconds elapsed'])

%% Controller parameters
display('Setting controller parameters')

controller_parameters = struct();

% Controller sampling time
controller_parameters.Ts = 2.5e-2; % 40Hz

% Controller weights
controller_parameters.tau = 1;
controller_parameters.C = [1 0 0 0; 0 0 0 0];
controller_parameters.D = [0; 1];
controller_parameters.W = @(vx)(diag([max(vx, 5) / 4, 10 * pi / 180])^-2); 

display(['    Completed, ' num2str(toc) ' seconds elapsed']);

%% Observer parameters
display('Setting observer parameters');

observer_parameters = struct();

% Observer sampling time
observer_parameters.Ts = controller_parameters.Ts;
observer_parameters.subsampling = 4; % 10 Hz

% Observer parameters
observer_parameters.rho = 1e-3;
observer_parameters.C = eye(2,4);

display(['    Completed, ' num2str(toc) ' seconds elapsed']);

%% Simulation parameters
display('Setting simulation parameters');

simulation_parameters = struct();

% Load trajectory
% simulation_parameters.trajectory = load_trajectory('bandejao.csv');
simulation_parameters.trajectory = load_trajectory('bloco_didatico.csv');

% Simulation initial state
simulation_parameters.x0 = [simulation_parameters.trajectory.s(1); 0; 0; 
                            simulation_parameters.trajectory.vx(1); 0; 0];
                        
simulation_parameters.t_end = sum(diff(simulation_parameters.trajectory.s) ./ ...
                                  simulation_parameters.trajectory.vx(1:end-1));

% GPS parameters
simulation_parameters.gps = struct();
simulation_parameters.gps.C = [zeros(4,1) eye(4,5)];
simulation_parameters.gps.Cov = diag([0.04, 0.5*pi/180, 0.006, 0.006]) ^ 2;

display(['    Completed, ' num2str(toc) ' seconds elapsed']);

%% Generate controller
display('Generating controllers');

controller = generate_controllers(vehicle_parameters, controller_parameters);

display(['    Completed, ' num2str(toc) ' seconds elapsed']);

%% Plot resulting controller frequency response
if is_plotting_results
    display('Plotting controller transfer functions');
    
    plot_lateral_controller(vehicle_parameters, controller_parameters, controller);
    
    display(['    Completed, ' num2str(toc) ' seconds elapsed']);
end

%% Generate observer
display('Generating robust observer w/ loop transfer recovery');

observer = generate_lateral_observer(vehicle_parameters, observer_parameters);

display(['    Completed, ' num2str(toc) ' seconds elapsed']);

%% Plot resulting observer frequency response
if is_plotting_results
    display('Plotting robust observer transfer functions');
    
    plot_lateral_observer(vehicle_parameters, controller_parameters, observer_parameters, ...
                          controller, observer);
    
    display(['    Completed, ' num2str(toc) ' seconds elapsed']);
end

%% Save controller and observer to MAT file
display('Generating controller and observer gains MAT file');

save('vehicle_code/controller_gains.mat', 'controller', 'observer')

display(['    Completed, ' num2str(toc) ' seconds elapsed']);

%% Finish initialization
display(['Initialization Completed in ' num2str(toc) ' seconds']);