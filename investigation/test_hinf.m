% Clean up
% clear all; close all; clc

display('Initializing Environment')

% Add subfolders to path
addpath('../models')
addpath('../scripts')
addpath('../sim_m_blocks')
addpath('..');

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
vehicle_parameters.brake.actuator_tc = 1;
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

%% Controller parameters
display('Setting controller parameters')

controller_parameters = struct();

% Controller sampling time
controller_parameters.Ts = 2e-2; % 50Hz

% Controller weights
controller_parameters.tau = 1;
controller_parameters.C = [1 0 0 0; 0 0 0 0];
controller_parameters.D = [0; 1];
controller_parameters.W = @(vx)(diag([max(vx,5) / 8, 10 * pi / 180])^-2); 

%% Get system matrix
display('Calculate system matrix for vx = 10 m/s')
vx = 8.11;

[A, Bu, Br, Hc, Ea, Ebu, Ebr] = ...
    linear_bicycle_model_uncertain(vx, [0.9, 0.9], vehicle_parameters);

[F, G, H, Ef, Eg] = lct.uc2d(A, Bu, 1000*Hc, Ea/1000, Ebu/1000, controller_parameters.Ts);
[~, ~, Gr, ~, ~] = lct.uc2d(A, Bu, Br, Ea, Ebu, controller_parameters.Ts);
Egr = Ebr/1000;

gamma = vx ^ 2 * 0.15 / 5;

%% Generate controller

Nx = size(F,2);
Nu = size(G,2);
Nr = size(Gr,2);
Nw = size(H,2);

Qp = controller_parameters.C;
Rp = controller_parameters.D;
tau = controller_parameters.tau;
W = controller_parameters.W;

% Initialize exponential convergent cost
sQp = size(Qp,1);
Rv = zeros(sQp * size(tau,1), Nu);
Qv = zeros(sQp * size(tau,1), Nx);

% Create exponential convergent basis
for j = 1:size(tau,1)
    Rv(sQp * (j - 1) + (1:sQp), :) = Qp * A^(j-1) * Bu + Rp;
    Qv(sQp * (j - 1) + (1:sQp), :) = Qp * A^(j-1) * (eye(Nx) / tau(j) + A);
end

% Create matrices Q and R
Q = Qv' * W(vx) * Qv;
R = Rv' * W(vx) * Rv;

C = [1 0 0 0];
D = 0;

[~, s, v] = svd(Q);
mask = (diag(s) >= 1e-10);

Qv = s(mask, mask);
Qp = v(:, mask)';

% Hinf?
Pinv = sdpvar(Nx, Nx);
KPinv = sdpvar(Nu, Nx, 'full');
Kr = sdpvar(Nu, Nr, 'full');

S = sdpvar(Nx, Nx);
e = sdpvar(1,1);
lambda = sdpvar(1,1);

M = blkvar;
M(1,1) = - eye(Nr);
M(1,3) = C * Pinv;
M(1,4) = D;
M(2,2) = - Pinv;
M(2,3) = F * Pinv - G * KPinv;
M(2,4) = Gr - G * Kr;
M(3,3) = - Pinv;
M(4,4) = - gamma^2 * eye(Nr);

N = blkvar;
N(1,1) = Pinv;
N(1,2) = eye(Nx);
N(2,2) = S;

constraints = [Pinv >= 0;
               Pinv <= 10 * eye(Nx);
               M <= 0;
               N >= 0];

opt = sdpsettings('solver', 'sedumi', 'verbose', 1, 'debug', 1);
sol = optimize(constraints, trace(S), opt)

if sol.problem ~= 0
    warning('gcc:solver_failed','SeDuMi did not converge');
end

K = value(KPinv) / value(Pinv);
Kr = value(Kr);
N = value(Kr);

P = inv(value(Pinv));
e = value(e);
lambda = value(lambda);

controller.lateral.vx = vx;
controller.lateral.vx_min = vx;
controller.lateral.vx_max = vx;
controller.lateral.K = [K Kr];
controller.lateral.P = P;