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
vehicle_parameters.steering.tc = 1/3;
vehicle_parameters.steering.rate_limit = 19.04 * pi / 180;
vehicle_parameters.steering.angle_limit = 24 * pi / 180;

% Brake System params
vehicle_parameters.brake.front_ratio = 0.7;
vehicle_parameters.brake.rear_ratio = 0.3;

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
controller_parameters.Ts = 2e-2; % 50Hz

% Controller weights
controller_parameters.tau = 1;
controller_parameters.C = [1 0 0 0; 0 0 0 0];
controller_parameters.D = [0; 1];
controller_parameters.W = @(vx)(diag([vx / 6, 10 * pi / 180])^-2); 

display(['    Completed, ' num2str(toc) ' seconds elapsed']);

%% Observer parameters
display('Setting observer parameters');

observer_parameters = struct();

% Observer sampling time
observer_parameters.Ts = controller_parameters.Ts;
observer_parameters.subsampling = 2;

% Observer parameters
observer_parameters.rho = 1e-4;
observer_parameters.C = eye(3,4);

display(['    Completed, ' num2str(toc) ' seconds elapsed']);

%% Simulation parameters
display('Setting simulation parameters');

simulation_parameters = struct();

% Simulation initial state
simulation_parameters.x0 = [0; 0; 0; 10; 0; 0];

% GPS parameters
simulation_parameters.gps = struct();
simulation_parameters.gps.C = [zeros(4,1) eye(4,5)];
simulation_parameters.gps.Cov = diag([0.04, 0.5*pi/180, 0.006, 0.006]) ^ 2;

display(['    Completed, ' num2str(toc) ' seconds elapsed']);

%% Generate controller
display('Generating robust controller');

controller = generate_lateral_controller(vehicle_parameters, controller_parameters);

display(['    Completed, ' num2str(toc) ' seconds elapsed']);

%% Plot resulting controller frequency response
if is_plotting_results
    display('Plotting robust controller transfer functions');
    
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

%% Finish initialization
display(['Initialization Completed in ' num2str(toc) ' seconds']);