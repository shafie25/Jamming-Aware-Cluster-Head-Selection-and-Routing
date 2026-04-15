%% run_routing_comparison.m — Dijkstra routing vs direct CH-to-BS
% Runs proposed scheme (Dijkstra) vs proposed scheme (direct) vs LEACH
% across 20 seeds. Isolates how much of the performance gap vs LEACH
% comes from JR-aware election alone vs election + routing.

clc; clear; close all;
addpath(genpath('.'));

seeds   = 1:20;
n_seeds = length(seeds);

config;

store = struct();
schemes = {'dijkstra', 'direct', 'leach'};
for k = 1:length(schemes)
    s = schemes{k};
    store.PDR.(s)    = zeros(n_seeds, T);
    store.energy.(s) = zeros(n_seeds, T);
    store.alive.(s)  = zeros(n_seeds, T);
    store.tdeath.(s) = zeros(n_seeds, 1);
end

for si = 1:n_seeds
    fprintf('=== Seed %d (%d/%d) ===\n', seeds(si), si, n_seeds);
    rng(seeds(si));
    config; init_network; uav_trajectory;

    %% Proposed + Dijkstra
    rng(seeds(si)); config; init_network; uav_trajectory;
    r = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
        E0, d_max, T, K_elec, M, lambda, p_CH, r_c, r_exc, ...
        alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
    store.PDR.dijkstra(si,:)    = r.PDR;
    store.energy.dijkstra(si,:) = r.energy;
    store.alive.dijkstra(si,:)  = r.alive;
    store.tdeath.dijkstra(si)   = r.t_death;
    fprintf('  Dijkstra  t_death=%d  PDR_all=%.2f%%  ZeroPDR=%d\n', ...
        r.t_death, mean(r.PDR)*100, sum(r.PDR==0));

    %% Proposed + Direct CH-to-BS
    rng(seeds(si)); config; init_network; uav_trajectory;
    r = run_proposed_direct(x, y, BS, J_x, J_y, dist_to_BS, ...
        E0, d_max, T, K_elec, M, lambda, p_CH, r_c, r_exc, ...
        alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
    store.PDR.direct(si,:)    = r.PDR;
    store.energy.direct(si,:) = r.energy;
    store.alive.direct(si,:)  = r.alive;
    store.tdeath.direct(si)   = r.t_death;
    fprintf('  Direct    t_death=%d  PDR_all=%.2f%%  ZeroPDR=%d\n', ...
        r.t_death, mean(r.PDR)*100, sum(r.PDR==0));

    %% LEACH
    rng(seeds(si)); config; init_network; uav_trajectory;
    r = run_leach(x, y, BS, J_x, J_y, E0, T, M, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
    store.PDR.leach(si,:)    = r.PDR;
    store.energy.leach(si,:) = r.energy;
    store.alive.leach(si,:)  = r.alive;
    store.tdeath.leach(si)   = r.t_death;
    fprintf('  LEACH     t_death=%d  PDR_all=%.2f%%  ZeroPDR=%d\n', ...
        r.t_death, mean(r.PDR)*100, sum(r.PDR==0));
end

%% Summary table
labels      = {'Proposed (Dijkstra)', 'Proposed (Direct)', 'LEACH'};
fields      = schemes;
n_schemes   = length(schemes);

fprintf('\n\n================================================================\n');
fprintf('Routing Comparison — 20-seed average\n');
fprintf('================================================================\n\n');
fprintf('%-22s', 'Metric');
for k = 1:n_schemes; fprintf(' | %-22s', labels{k}); end
fprintf('\n%s\n', repmat('-', 1, 22 + n_schemes*25));

for mi = 1:5
    switch mi
        case 1; label = 'First death (rnd)';   variant = 'tdeath';
        case 2; label = 'PDR all rounds (%)';  variant = 'all';
        case 3; label = 'PDR FND-trunc (%)';   variant = 'fnd';
        case 4; label = 'Zero-PDR rounds';     variant = 'zero';
        case 5; label = 'Energy@r300 (J)';     variant = 'r300';
    end
    fprintf('%-22s', label);
    for k = 1:n_schemes
        fd = fields{k};
        switch variant
            case 'tdeath'
                v = store.tdeath.(fd);
                fprintf(' | %7.1f +/- %5.1f      ', mean(v,'omitnan'), std(v,'omitnan'));
            case 'all'
                pm = mean(store.PDR.(fd), 2) * 100;
                fprintf(' | %7.2f +/- %5.2f      ', mean(pm), std(pm));
            case 'fnd'
                pm = zeros(n_seeds,1);
                for s = 1:n_seeds
                    td = store.tdeath.(fd)(s);
                    if ~isnan(td) && td >= 1
                        pm(s) = mean(store.PDR.(fd)(s,1:round(td)))*100;
                    else
                        pm(s) = mean(store.PDR.(fd)(s,:))*100;
                    end
                end
                fprintf(' | %7.2f +/- %5.2f      ', mean(pm), std(pm));
            case 'zero'
                zp = sum(store.PDR.(fd) == 0, 2);
                fprintf(' | %7.1f +/- %5.1f      ', mean(zp), std(zp));
            case 'r300'
                em = store.energy.(fd)(:,300);
                fprintf(' | %7.2f +/- %5.2f      ', mean(em), std(em));
        end
    end
    fprintf('\n');
end
fprintf('%s\n', repmat('-', 1, 22 + n_schemes*25));
