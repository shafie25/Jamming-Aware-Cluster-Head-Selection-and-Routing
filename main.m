%% main.m — Simulation Entry Point
% Seeds RNG, initializes network and trajectory, runs proposed scheme,
% and calls plot_results. Baselines will be added here later.

clc; clear; close all;

%% Reproducibility
rng(42);   % fixed seed — all schemes see identical network and packet draws

%% Load Parameters
config;

%% Initialize Network
init_network;
% Workspace now contains: x, y, dist_to_BS, energy, PDR_ewma, JR, alive, d_max

%% Precompute UAV Trajectory
uav_trajectory;
% Workspace now contains: J_x, J_y (vectors of length T)

%% Run Proposed Scheme
fprintf('Running proposed scheme...\n');
results_proposed = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
    E0, d_max, T, K_elec, M, lambda, r_c, r_exc, ...
    alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L);

fprintf('First node death at round: %d\n', results_proposed.t_death);
fprintf('Done.\n');

%% Plot Results
results_all = {results_proposed};   % add baseline results here later
plot_results(results_all, T);