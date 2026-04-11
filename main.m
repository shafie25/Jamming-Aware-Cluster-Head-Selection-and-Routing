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
    p_base, kappa, r_j, E_elec, E_amp, E_da, L);
fprintf('  Proposed      first node death: round %d\n', results_proposed.t_death);

%% Run Standard LEACH
fprintf('Running standard LEACH...\n');
results_leach = run_leach(x, y, BS, J_x, J_y, E0, T, M, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L);
fprintf('  LEACH         first node death: round %d\n', results_leach.t_death);

%% Run Baseline 1 — EWMA Detection Only
fprintf('Running EWMA-Detect baseline...\n');
results_ewma = run_ewma_detect(x, y, BS, J_x, J_y, E0, T, M, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L, lambda);
fprintf('  EWMA-Detect   first node death: round %d\n', results_ewma.t_death);

%% Run Baseline 2 — Threshold-Based Suppression
fprintf('Running Threshold-JR baseline...\n');
results_threshold = run_threshold(x, y, BS, J_x, J_y, E0, T, M, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L, lambda);
fprintf('  Threshold-JR  first node death: round %d\n', results_threshold.t_death);

%% Run Baseline 3 — Reactive CH Re-election
fprintf('Running Reactive-CH baseline...\n');
results_reactive = run_reactive_ch(x, y, BS, J_x, J_y, E0, T, M, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L, lambda);
fprintf('  Reactive-CH   first node death: round %d\n', results_reactive.t_death);

fprintf('Done.\n');

%% Plot Results
results_all = {results_proposed, results_leach, results_ewma, results_threshold, results_reactive};
plot_results(results_all, T);
