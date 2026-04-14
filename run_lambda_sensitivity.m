%% run_lambda_sensitivity.m — Lambda Sensitivity Test (Proposed Scheme Only)
% Runs the proposed scheme across 5 seeds for lambda = 0.6, 0.7, 0.8
% and reports all three PDR windows side by side.

clc; clear; close all;
addpath(genpath('.'));

seeds        = [42, 7, 13, 99, 101];
n_seeds      = length(seeds);
lambdas      = [0.6, 0.7, 0.8];
n_lambdas    = length(lambdas);

%% Pre-allocate storage
config;   % load T and all other params

store = struct();
for li = 1:n_lambdas
    lbl = sprintf('l%d', li);
    store.PDR.(lbl)    = zeros(n_seeds, T);
    store.energy.(lbl) = zeros(n_seeds, T);
    store.alive.(lbl)  = zeros(n_seeds, T);
    store.tdeath.(lbl) = zeros(n_seeds, 1);
end

%% Run
for s = 1:n_seeds
    fprintf('=== Seed %d (%d/%d) ===\n', seeds(s), s, n_seeds);
    rng(seeds(s));
    config;
    init_network;
    uav_trajectory;

    for li = 1:n_lambdas
        lam = lambdas(li);
        lbl = sprintf('l%d', li);

        rng(seeds(s));   % reset RNG so packet draws are identical across lambdas
        config;
        init_network;
        uav_trajectory;

        rp = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
            E0, d_max, T, K_elec, M, lam, r_c, r_exc, ...
            alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
            p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);

        store.PDR.(lbl)(s,:)    = rp.PDR;
        store.energy.(lbl)(s,:) = rp.energy;
        store.alive.(lbl)(s,:)  = rp.alive;
        store.tdeath.(lbl)(s)   = rp.t_death;
        fprintf('  lambda=%.1f  t_death=%d  PDR_mean=%.2f%%  ZeroPDR=%d\n', ...
            lam, rp.t_death, mean(rp.PDR)*100, sum(rp.PDR==0));
    end
end

%% Summary table
fprintf('\n\n========================================================\n');
fprintf('Lambda Sensitivity — Proposed Scheme (5-seed average)\n');
fprintf('========================================================\n\n');
fprintf('%-22s | %-18s | %-18s | %-18s\n', 'Metric', ...
    sprintf('lambda=%.1f', lambdas(1)), ...
    sprintf('lambda=%.1f', lambdas(2)), ...
    sprintf('lambda=%.1f', lambdas(3)));
fprintf('%s\n', repmat('-', 1, 82));

metrics = {
    'First death (rnd)',   'tdeath', '';
    'PDR all rounds (%)',  'PDR',    'all';
    'PDR FND-trunc (%)',   'PDR',    'fnd';
    'Zero-PDR rounds',     'PDR',    'zero';
    'Energy@r300 (J)',     'energy', 'r300';
};

for mi = 1:size(metrics, 1)
    label   = metrics{mi, 1};
    field   = metrics{mi, 2};
    variant = metrics{mi, 3};

    fprintf('%-22s |', label);
    for li = 1:n_lambdas
        lbl = sprintf('l%d', li);
        switch variant
            case ''   % t_death
                vals = store.tdeath.(lbl);
                m = mean(vals, 'omitnan'); s_ = std(vals, 'omitnan');
                fprintf(' %6.1f +/- %5.1f      |', m, s_);
            case 'all'
                pm = mean(store.PDR.(lbl), 2) * 100;
                fprintf(' %6.2f +/- %5.2f      |', mean(pm), std(pm));
            case 'fnd'
                pm_fnd = zeros(n_seeds, 1);
                for s = 1:n_seeds
                    td = store.tdeath.(lbl)(s);
                    if ~isnan(td) && td >= 1
                        pm_fnd(s) = mean(store.PDR.(lbl)(s, 1:round(td))) * 100;
                    else
                        pm_fnd(s) = mean(store.PDR.(lbl)(s,:)) * 100;
                    end
                end
                fprintf(' %6.2f +/- %5.2f      |', mean(pm_fnd), std(pm_fnd));
            case 'zero'
                zp = sum(store.PDR.(lbl) == 0, 2);
                fprintf(' %6.1f +/- %5.1f      |', mean(zp), std(zp));
            case 'r300'
                em = store.energy.(lbl)(:, 300);
                fprintf(' %6.2f +/- %5.2f      |', mean(em), std(em));
        end
    end
    fprintf('\n');
end
fprintf('%s\n', repmat('-', 1, 82));
