%% run_multiseed.m — Multi-Seed Averaging Entry Point
% Runs all five schemes across multiple RNG seeds and reports mean ± std.
% Each seed produces a different network topology and packet sequence.
%
% Schemes evaluated:
%   1. Proposed      — JR-aware CH election (CHScore) + JR-aware Dijkstra routing
%   2. LEACH         — Standard LEACH, no jamming awareness
%   3. EWMA-Detect   — LEACH + EWMA detection (detection computed, not used)
%   4. Threshold-JR  — LEACH + jammed member suppression (JR > 0.7)
%   5. Reactive-CH   — LEACH + reactive CH re-election when CH JR > 0.5
%
% Run this for final reported results. Use main.m for quick single-seed checks.

clc; clear; close all;
addpath(genpath('.'));

seeds   = [42, 7, 13, 99, 101];
n_seeds = length(seeds);
n_schemes = 5;

%% Pre-allocate storage (need T — load config once)
config;

store = struct();
fields = {'PDR','energy','delay','alive'};
labels = {'proposed','leach','ewma','threshold','reactive'};

for f = 1:length(fields)
    for l = 1:length(labels)
        store.(fields{f}).(labels{l}) = zeros(n_seeds, T);
    end
end
tdeath = zeros(n_seeds, n_schemes);   % cols: proposed, leach, ewma, threshold, reactive

%% Run all schemes for each seed
for s = 1:n_seeds
    fprintf('=== Seed %d (%d/%d) ===\n', seeds(s), s, n_seeds);
    rng(seeds(s));

    config;
    init_network;
    uav_trajectory;

    %% Proposed
    rp = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
        E0, d_max, T, K_elec, M, lambda, r_c, r_exc, ...
        alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L);
    store.PDR.proposed(s,:)    = rp.PDR;
    store.energy.proposed(s,:) = rp.energy;
    store.delay.proposed(s,:)  = rp.delay;
    store.alive.proposed(s,:)  = rp.alive;
    tdeath(s,1) = rp.t_death;
    fprintf('  Proposed     t_death=%d\n', rp.t_death);

    %% Standard LEACH
    rl = run_leach(x, y, BS, J_x, J_y, E0, T, M, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L);
    store.PDR.leach(s,:)    = rl.PDR;
    store.energy.leach(s,:) = rl.energy;
    store.delay.leach(s,:)  = rl.delay;
    store.alive.leach(s,:)  = rl.alive;
    tdeath(s,2) = rl.t_death;
    fprintf('  LEACH        t_death=%d\n', rl.t_death);

    %% Baseline 1 — EWMA Detection Only
    re = run_ewma_detect(x, y, BS, J_x, J_y, E0, T, M, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L, lambda);
    store.PDR.ewma(s,:)    = re.PDR;
    store.energy.ewma(s,:) = re.energy;
    store.delay.ewma(s,:)  = re.delay;
    store.alive.ewma(s,:)  = re.alive;
    tdeath(s,3) = re.t_death;
    fprintf('  EWMA-Detect  t_death=%d\n', re.t_death);

    %% Baseline 2 — Threshold-Based Suppression
    rt = run_threshold(x, y, BS, J_x, J_y, E0, T, M, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L, lambda);
    store.PDR.threshold(s,:)    = rt.PDR;
    store.energy.threshold(s,:) = rt.energy;
    store.delay.threshold(s,:)  = rt.delay;
    store.alive.threshold(s,:)  = rt.alive;
    tdeath(s,4) = rt.t_death;
    fprintf('  Threshold-JR t_death=%d\n', rt.t_death);

    %% Baseline 3 — Reactive CH Re-election
    rr = run_reactive_ch(x, y, BS, J_x, J_y, E0, T, M, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L, lambda);
    store.PDR.reactive(s,:)    = rr.PDR;
    store.energy.reactive(s,:) = rr.energy;
    store.delay.reactive(s,:)  = rr.delay;
    store.alive.reactive(s,:)  = rr.alive;
    tdeath(s,5) = rr.t_death;
    fprintf('  Reactive-CH  t_death=%d\n', rr.t_death);
end

%% Compute summary statistics
scheme_names  = {'Proposed','LEACH','EWMA-Detect','Threshold-JR','Reactive-CH'};
scheme_fields = labels;

fprintf('\n\n========================================\n');
fprintf('Multi-Seed Results (seeds: %s)\n', num2str(seeds));
fprintf('========================================\n\n');

fprintf('%-16s | %-20s | %-20s | %-20s | %-20s | %-20s\n', ...
    'Metric', scheme_names{:});
fprintf('%s\n', repmat('-',1,120));

% First node death
row = 'First death (rnd)';
fprintf('%-16s |', row);
for k = 1:n_schemes
    td = tdeath(:,k);
    fprintf(' %6.1f +/- %5.1f      |', mean(td,'omitnan'), std(td,'omitnan'));
end
fprintf('\n');

% PDR mean (all rounds)
row = 'PDR mean (%)';
fprintf('%-16s |', row);
for k = 1:n_schemes
    fd = scheme_fields{k};
    pm = mean(store.PDR.(fd), 2) * 100;
    fprintf(' %6.2f +/- %5.2f      |', mean(pm), std(pm));
end
fprintf('\n');

% Energy @ round 300
row = 'Energy@r300 (J)';
fprintf('%-16s |', row);
for k = 1:n_schemes
    fd = scheme_fields{k};
    em = store.energy.(fd)(:,300);
    fprintf(' %6.2f +/- %5.2f      |', mean(em), std(em));
end
fprintf('\n');

% Zero-PDR rounds
row = 'Zero-PDR rounds';
fprintf('%-16s |', row);
for k = 1:n_schemes
    fd = scheme_fields{k};
    zp = sum(store.PDR.(fd) == 0, 2);
    fprintf(' %6.1f +/- %5.1f      |', mean(zp), std(zp));
end
fprintf('\n');
fprintf('%s\n', repmat('-',1,120));

%% Build structs for plotting
ms = cell(1, n_schemes);
for k = 1:n_schemes
    fd = scheme_fields{k};
    ms{k}.PDR_mean    = mean(store.PDR.(fd),    1);
    ms{k}.PDR_std     = std( store.PDR.(fd),  0, 1);
    ms{k}.energy_mean = mean(store.energy.(fd), 1);
    ms{k}.energy_std  = std( store.energy.(fd),0, 1);
    ms{k}.delay_mean  = mean(store.delay.(fd),  1);
    ms{k}.delay_std   = std( store.delay.(fd), 0, 1);
    ms{k}.alive_mean  = mean(store.alive.(fd),  1);
    ms{k}.alive_std   = std( store.alive.(fd), 0, 1);
    ms{k}.label       = scheme_names{k};
end

plot_multiseed(ms, T);
