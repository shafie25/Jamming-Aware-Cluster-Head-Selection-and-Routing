%% main.m — Simulation Entry Point (Single Seed)
% Seeds RNG, initializes network and trajectory, runs all five schemes,
% and plots results. Use run_multiseed.m for final averaged results.

clc; clear; close all;
addpath(genpath('.'));

%% Reproducibility
rng(42);   % fixed seed — all schemes see identical network and packet draws

%% Load Parameters
config;

%% Initialize Network
init_network;

%% Precompute UAV Trajectory
uav_trajectory;

%% Run Proposed Scheme
fprintf('Running proposed scheme...\n');
results_proposed = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
    E0, d_max, T, K_elec, M, lambda, r_c, r_exc, ...
    alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
fprintf('  Proposed      first node death: round %d\n', results_proposed.t_death);

%% Run Standard LEACH
fprintf('Running standard LEACH...\n');
results_leach = run_leach(x, y, BS, J_x, J_y, E0, T, M, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
fprintf('  LEACH         first node death: round %d\n', results_leach.t_death);

fprintf('Done.\n');

%% Plot Results
results_all = {results_proposed, results_leach};
plot_results(results_all, T);
