%% run_phi1_sweep.m — phi1 (per-hop penalty) Sensitivity Test
% Sweeps phi1 across {1e-4, 3e-4, 5e-4, 1e-3} for the proposed scheme only.
% All other parameters held at canonical Run 013 values.
% Reports all three PDR windows + first-death + energy side by side.

clc; clear; close all;
addpath(genpath('.'));

seeds     = 1:20;
n_seeds   = length(seeds);
phi1_vals = [1e-4, 3e-4, 5e-4, 1e-3];
n_phi1    = length(phi1_vals);

%% Pre-allocate storage
config;

store = struct();
for pi = 1:n_phi1
    lbl = sprintf('p%d', pi);
    store.PDR.(lbl)    = zeros(n_seeds, T);
    store.energy.(lbl) = zeros(n_seeds, T);
    store.alive.(lbl)  = zeros(n_seeds, T);
    store.tdeath.(lbl) = zeros(n_seeds, 1);
end

%% Run
for s = 1:n_seeds
    fprintf('=== Seed %d (%d/%d) ===\n', seeds(s), s, n_seeds);

    for pi = 1:n_phi1
        lbl  = sprintf('p%d', pi);
        phi1_test = phi1_vals(pi);

        rng(seeds(s));
        config;
        init_network;
        uav_trajectory;

        rp = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
            E0, d_max, T, K_elec, M, lambda, p_CH, r_c, r_exc, ...
            alpha, beta, gamma_, delta, phi1_test, phi2, phi3, ...
            p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);

        store.PDR.(lbl)(s,:)    = rp.PDR;
        store.energy.(lbl)(s,:) = rp.energy;
        store.alive.(lbl)(s,:)  = rp.alive;
        store.tdeath.(lbl)(s)   = rp.t_death;
        fprintf('  phi1=%.0e  t_death=%d  PDR_all=%.2f%%  PDR_fnd=%.2f%%  ZeroPDR=%d\n', ...
            phi1_test, rp.t_death, mean(rp.PDR)*100, ...
            mean(rp.PDR(1:max(1,round(rp.t_death))))*100, sum(rp.PDR==0));
    end
end

%% Summary table
fprintf('\n\n================================================================\n');
fprintf('phi1 Sensitivity — Proposed Scheme (20-seed average)\n');
fprintf('================================================================\n\n');
fprintf('%-22s', 'Metric');
for pi = 1:n_phi1
    fprintf(' | %-20s', sprintf('phi1=%.0e', phi1_vals(pi)));
end
fprintf('\n%s\n', repmat('-', 1, 22 + n_phi1*23));

metrics = {
    'First death (rnd)',  'tdeath', '';
    'PDR all rounds (%)', 'PDR',    'all';
    'PDR FND-trunc (%)',  'PDR',    'fnd';
    'Zero-PDR rounds',    'PDR',    'zero';
    'Energy@r300 (J)',    'energy', 'r300';
    'Avg hops (r1-300)',  'alive',  'hops';
};

for mi = 1:size(metrics, 1)
    label   = metrics{mi, 1};
    field   = metrics{mi, 2};
    variant = metrics{mi, 3};

    if strcmp(label, 'Avg hops (r1-300)'); continue; end   % skip — not stored here

    fprintf('%-22s', label);
    for pi = 1:n_phi1
        lbl = sprintf('p%d', pi);
        switch variant
            case ''
                vals = store.tdeath.(lbl);
                m = mean(vals, 'omitnan'); s_ = std(vals, 'omitnan');
                fprintf(' | %6.1f +/- %5.1f      ', m, s_);
            case 'all'
                pm = mean(store.PDR.(lbl), 2) * 100;
                fprintf(' | %6.2f +/- %5.2f      ', mean(pm), std(pm));
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
                fprintf(' | %6.2f +/- %5.2f      ', mean(pm_fnd), std(pm_fnd));
            case 'zero'
                zp = sum(store.PDR.(lbl) == 0, 2);
                fprintf(' | %6.1f +/- %5.1f      ', mean(zp), std(zp));
            case 'r300'
                em = store.energy.(lbl)(:, 300);
                fprintf(' | %6.2f +/- %5.2f      ', mean(em), std(em));
        end
    end
    fprintf('\n');
end
fprintf('%s\n', repmat('-', 1, 22 + n_phi1*23));

%% Plot: mean PDR curves for all phi1 values
rounds = 1:T;
colors = {[0.18,0.45,0.69], [0.20,0.63,0.17], [0.89,0.47,0.07], [0.80,0.15,0.15]};

figure('Position', [100, 100, 1200, 450]);

subplot(1,2,1); hold on;
for pi = 1:n_phi1
    lbl = sprintf('p%d', pi);
    c   = colors{pi};
    mu  = mean(store.PDR.(lbl), 1);
    sg  = std( store.PDR.(lbl), 0, 1);
    fill([rounds, fliplr(rounds)], [min(mu+sg,1), fliplr(max(mu-sg,0))], ...
        c, 'FaceAlpha', 0.10, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(rounds, mu, 'Color', c, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('\\phi_1 = %.0e', phi1_vals(pi)));
end
xlabel('Round'); ylabel('PDR');
title('PDR — phi1 sweep (mean \pm std, 20 seeds)');
legend('Location', 'southwest', 'FontSize', 8); grid on; ylim([0 1]);

subplot(1,2,2); hold on;
for pi = 1:n_phi1
    lbl = sprintf('p%d', pi);
    c   = colors{pi};
    mu  = mean(store.energy.(lbl), 1);
    sg  = std( store.energy.(lbl), 0, 1);
    fill([rounds, fliplr(rounds)], [mu+sg, fliplr(max(mu-sg,0))], ...
        c, 'FaceAlpha', 0.10, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(rounds, mu, 'Color', c, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('\\phi_1 = %.0e', phi1_vals(pi)));
end
xlabel('Round'); ylabel('Total Residual Energy (J)');
title('Network Energy — phi1 sweep (mean \pm std, 20 seeds)');
legend('Location', 'northeast', 'FontSize', 8); grid on;

sgtitle('phi1 Per-Hop Penalty Sensitivity — Proposed Scheme');
