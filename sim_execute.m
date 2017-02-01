% sim_execute Main simulation script
%
%    Author: Carlos M. Massera
%    Instituition: University of São Paulo

%% Remove the polytope from controller struct (Simulink doesn't like it)
controller_full = controller;
controller.lateral.inv_sets = 0;
controller.lateral.offset_opts = 0;

offset_optimization = controller_full.lateral.offset_opts;

%% Run GCC simulation
options = simset('SrcWorkspace','current');
sim_out = sim('sim_environment_gcc', [], options);

log_out = sim_out.get('logsout');

%% Plot GCC simulation results
plot_sim_results(log_out, vehicle_parameters, 'sim_gcc_results');

%% Run LTR-GCC simulation
options = simset('SrcWorkspace','current');
sim_out = sim('sim_environment_ltr_gcc', [], options);

log_out = sim_out.get('logsout');

%% Plot LTR-GCC simulation results
plot_sim_results(log_out, vehicle_parameters, 'sim_ltr_gcc_results');

%% Recover controller variables
controller = controller_full;