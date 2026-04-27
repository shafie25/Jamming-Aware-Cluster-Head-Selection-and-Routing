%% run_multiseed.m — Multi-Seed Averaging Entry Point
% Runs proposed scheme, TBC, and FCPA across multiple RNG seeds
% and reports mean ± std. Each seed produces a different network topology
% and packet sequence.
%
% Run this for final reported results. Use main.m for quick single-seed checks.

clc; clear; close all;
addpath(genpath('.'));

seeds   = 1:100;
n_seeds = length(seeds);
n_schemes = 3;

%% Pre-allocate storage (need T, N — load config once)
config;

store = struct();
fields = {'PDR','energy','delay','alive'};
labels = {'proposed','tbc','fcpa'};

for f = 1:length(fields)
    for l = 1:length(labels)
        store.(fields{f}).(labels{l}) = zeros(n_seeds, T);
    end
end
tdeath = zeros(n_seeds, n_schemes);   % cols: proposed, tbc, fcpa

%% Run all schemes for each seed
for s = 1:n_seeds
    fprintf('=== Seed %d (%d/%d) ===\n', seeds(s), s, n_seeds);
    rng(seeds(s));

    config;
    init_network;
    uav_trajectory;

    %% Proposed
    rp = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
        E0, d_max, T, K_elec, M, lambda, p_CH, r_c, r_exc, ...
        alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
    store.PDR.proposed(s,:)    = rp.PDR;
    store.energy.proposed(s,:) = rp.energy;
    store.delay.proposed(s,:)  = rp.delay;
    store.alive.proposed(s,:)  = rp.alive;
    tdeath(s,1) = rp.t_death;
    fprintf('  Proposed     t_death=%d\n', rp.t_death);

    %% TBC Baseline
    rt = run_tbc(x, y, BS, J_x, J_y, ...
        E0, T, M, p_base, kappa, r_j, E_elec, E_amp, L, r_tx);
    store.PDR.tbc(s,:)    = rt.PDR;
    store.energy.tbc(s,:) = rt.energy;
    store.delay.tbc(s,:)  = rt.delay;
    store.alive.tbc(s,:)  = rt.alive;
    tdeath(s,2) = rt.t_death;
    fprintf('  TBC          t_death=%d\n', rt.t_death);

    %% FCPA Baseline
    rf = run_fcpa(x, y, BS, J_x, J_y, ...
        E0, T, M, K_elec, p_CH, p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
    store.PDR.fcpa(s,:)    = rf.PDR;
    store.energy.fcpa(s,:) = rf.energy;
    store.delay.fcpa(s,:)  = rf.delay;
    store.alive.fcpa(s,:)  = rf.alive;
    tdeath(s,3) = rf.t_death;
    fprintf('  FCPA         t_death=%d\n', rf.t_death);
end

%% Compute summary statistics
scheme_names  = {'Proposed','TBC','FCPA'};
scheme_fields = labels;

fprintf('\n\n========================================\n');
fprintf('Multi-Seed Results (seeds: %s)\n', num2str(seeds));
fprintf('========================================\n\n');

fprintf('%-22s | %-20s | %-20s | %-20s\n', 'Metric', scheme_names{:});
fprintf('%s\n', repmat('-',1,92));

% First node death (FND)
fprintf('%-22s |', 'FND (rnd)');
for k = 1:n_schemes
    td = tdeath(:,k);
    fprintf(' %6.1f +/- %5.1f      |', mean(td,'omitnan'), std(td,'omitnan'));
end
fprintf('\n');

% Half-network death (HND): first round when alive <= N/2
fprintf('%-22s |', 'HND (rnd)');
for k = 1:n_schemes
    fd = scheme_fields{k};
    hnd = zeros(n_seeds, 1);
    for s = 1:n_seeds
        alive_s  = store.alive.(fd)(s,:);
        half_idx = find(alive_s <= N/2, 1, 'first');
        hnd(s)   = half_idx;   % NaN if never reached (half_idx empty → MATLAB returns [])
        if isempty(half_idx); hnd(s) = T; end
    end
    fprintf(' %6.1f +/- %5.1f      |', mean(hnd,'omitnan'), std(hnd,'omitnan'));
end
fprintf('\n');

% --- PDR Window 1: All T rounds ---
fprintf('%-22s |', 'PDR all rounds (%)');
for k = 1:n_schemes
    fd = scheme_fields{k};
    pm = mean(store.PDR.(fd), 2) * 100;
    fprintf(' %6.2f +/- %5.2f      |', mean(pm), std(pm));
end
fprintf('\n');

% --- PDR Window 2: FND-truncated ---
fprintf('%-22s |', 'PDR FND-trunc (%)');
for k = 1:n_schemes
    fd = scheme_fields{k};
    pm_fnd = zeros(n_seeds, 1);
    for s = 1:n_seeds
        td = tdeath(s, k);
        if ~isnan(td) && td >= 1
            pm_fnd(s) = mean(store.PDR.(fd)(s, 1:round(td))) * 100;
        else
            pm_fnd(s) = mean(store.PDR.(fd)(s, :)) * 100;
        end
    end
    fprintf(' %6.2f +/- %5.2f      |', mean(pm_fnd), std(pm_fnd));
end
fprintf('\n');

% --- PDR @ round 300 (all schemes fully alive — window-artefact-free snapshot) ---
fprintf('%-22s |', 'PDR@r300 (%)');
for k = 1:n_schemes
    fd = scheme_fields{k};
    pm300 = store.PDR.(fd)(:, 300) * 100;
    fprintf(' %6.2f +/- %5.2f      |', mean(pm300), std(pm300));
end
fprintf('\n');

% Energy @ round 300
fprintf('%-22s |', 'Energy@r300 (J)');
for k = 1:n_schemes
    fd = scheme_fields{k};
    em = store.energy.(fd)(:,300);
    fprintf(' %6.2f +/- %5.2f      |', mean(em), std(em));
end
fprintf('\n');

% --- Total packets delivered: sum_t( PDR(t) * alive(t) * M ) in thousands ---
fprintf('%-22s |', 'Total del. pkts (k)');
for k = 1:n_schemes
    fd = scheme_fields{k};
    tpd = zeros(n_seeds, 1);
    for s = 1:n_seeds
        tpd(s) = sum(store.PDR.(fd)(s,:) .* store.alive.(fd)(s,:)) * M / 1000;
    end
    fprintf(' %6.1f +/- %5.1f      |', mean(tpd), std(tpd));
end
fprintf('\n');

fprintf('%s\n', repmat('-',1,92));

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

% Pre-compute per-seed total delivered packets for plotting struct
for k = 1:n_schemes
    fd = scheme_fields{k};
    tpd_all = zeros(n_seeds, T);
    for s = 1:n_seeds
        tpd_all(s,:) = cumsum(store.PDR.(fd)(s,:) .* store.alive.(fd)(s,:)) * M;
    end
    ms{k}.cum_pkts_mean = mean(tpd_all, 1);
    ms{k}.cum_pkts_std  = std( tpd_all, 0, 1);
end

plot_multiseed(ms, T);
