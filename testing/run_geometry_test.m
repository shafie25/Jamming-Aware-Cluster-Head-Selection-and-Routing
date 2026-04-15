%% run_geometry_test.m — BS Position Geometry Test
% Tests whether Dijkstra inter-cluster routing provides meaningful benefit
% when the BS is NOT at the field center. Three BS positions are tested:
%
%   Center [50, 50]  — baseline (routing ~irrelevant, ~78% of field in BS range)
%   Edge   [50,  0]  — BS on bottom edge (routing helps nodes in top half)
%   Corner [ 0,  0]  — BS at corner (routing critical; ~19% of field in range)
%
% For each BS position: Proposed (Dijkstra) vs Proposed (Direct) vs LEACH.
% Reports PDR gain from routing = PDR(Dijkstra) - PDR(Direct) per geometry.
%
% Run from project root: >> cd <project_root>; run_geometry_test

clc; clear; close all;
addpath(genpath('.'));

seeds   = 1:20;
n_seeds = length(seeds);

%% BS configurations to test
bs_list = {
    [50, 50], 'Center [50,50]';
    [50,  0], 'Edge   [50,0]';
    [ 0,  0], 'Corner [0,0]';
};
n_bs = size(bs_list, 1);

schemes    = {'dijkstra', 'direct', 'leach'};
n_schemes  = length(schemes);

%% Pre-allocate
config;
store = struct();
for bi = 1:n_bs
    bk = sprintf('bs%d', bi);
    for k = 1:n_schemes
        sk = schemes{k};
        store.(bk).(sk).PDR    = zeros(n_seeds, T);
        store.(bk).(sk).energy = zeros(n_seeds, T);
        store.(bk).(sk).tdeath = zeros(n_seeds, 1);
    end
end

%% Run
for bi = 1:n_bs
    BS_test = bs_list{bi, 1};
    bs_label = bs_list{bi, 2};
    bk = sprintf('bs%d', bi);

    fprintf('\n========================================\n');
    fprintf('BS position: %s\n', bs_label);
    fprintf('========================================\n');

    for si = 1:n_seeds
        fprintf('  Seed %d/%d\n', si, n_seeds);

        %% Proposed + Dijkstra
        rng(seeds(si)); config; init_network; uav_trajectory;
        BS = BS_test;
        dist_to_BS = sqrt((x - BS(1)).^2 + (y - BS(2)).^2);
        r = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
            E0, d_max, T, K_elec, M, lambda, p_CH, r_c, r_exc, ...
            alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
            p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
        store.(bk).dijkstra.PDR(si,:)    = r.PDR;
        store.(bk).dijkstra.energy(si,:) = r.energy;
        store.(bk).dijkstra.tdeath(si)   = r.t_death;

        %% Proposed + Direct
        rng(seeds(si)); config; init_network; uav_trajectory;
        BS = BS_test;
        dist_to_BS = sqrt((x - BS(1)).^2 + (y - BS(2)).^2);
        r = run_proposed_direct(x, y, BS, J_x, J_y, dist_to_BS, ...
            E0, d_max, T, K_elec, M, lambda, p_CH, r_c, r_exc, ...
            alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
            p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
        store.(bk).direct.PDR(si,:)    = r.PDR;
        store.(bk).direct.energy(si,:) = r.energy;
        store.(bk).direct.tdeath(si)   = r.t_death;

        %% LEACH
        rng(seeds(si)); config; init_network; uav_trajectory;
        BS = BS_test;
        r = run_leach(x, y, BS, J_x, J_y, E0, T, M, ...
            p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
        store.(bk).leach.PDR(si,:)    = r.PDR;
        store.(bk).leach.energy(si,:) = r.energy;
        store.(bk).leach.tdeath(si)   = r.t_death;
    end
end

%% Print summary table
fprintf('\n\n================================================================\n');
fprintf('Geometry Test Results (20-seed average)\n');
fprintf('================================================================\n\n');

for bi = 1:n_bs
    bk = sprintf('bs%d', bi);
    bs_label = bs_list{bi, 2};

    % Compute coverage: fraction of 100x100 field within r_tx of BS_test
    BS_test = bs_list{bi, 1};
    config;
    coverage_pct = pi * r_tx^2 / area^2 * 100;   % approximate circle area
    % Clamp circle at field boundary for corner/edge cases
    % (rough estimate — actual is less for corner/edge)
    if BS_test(1) == 0 || BS_test(1) == area; coverage_pct = coverage_pct / 2; end
    if BS_test(2) == 0 || BS_test(2) == area; coverage_pct = coverage_pct / 2; end

    fprintf('--- BS: %s  (approx %.0f%% of field within r_tx=%dm of BS) ---\n', ...
        bs_label, coverage_pct, r_tx);
    fprintf('  %-22s | %-12s | %-12s | %-12s | %s\n', ...
        'Metric', 'Dijkstra', 'Direct', 'LEACH', 'Routing gain');
    fprintf('  %s\n', repmat('-', 1, 80));

    metrics = {'PDR all (%)','PDR FND (%)','ZeroPDR','t_death','E@r300'};
    for mi = 1:length(metrics)
        vals = zeros(n_seeds, n_schemes);
        for k = 1:n_schemes
            sk = schemes{k};
            switch mi
                case 1
                    vals(:,k) = mean(store.(bk).(sk).PDR, 2) * 100;
                case 2
                    for s = 1:n_seeds
                        td = store.(bk).(sk).tdeath(s);
                        if ~isnan(td) && td >= 1
                            vals(s,k) = mean(store.(bk).(sk).PDR(s,1:round(td)))*100;
                        else
                            vals(s,k) = mean(store.(bk).(sk).PDR(s,:))*100;
                        end
                    end
                case 3
                    vals(:,k) = sum(store.(bk).(sk).PDR == 0, 2);
                case 4
                    vals(:,k) = store.(bk).(sk).tdeath;
                case 5
                    vals(:,k) = store.(bk).(sk).energy(:, 300);
            end
        end
        m = mean(vals, 1, 'omitnan');
        s_ = std(vals, 0, 1, 'omitnan');
        routing_gain = m(1) - m(2);   % Dijkstra minus Direct
        gain_str = sprintf('%+.2f', routing_gain);

        fprintf('  %-22s | %6.1f+/-%4.1f | %6.1f+/-%4.1f | %6.1f+/-%4.1f | %s\n', ...
            metrics{mi}, m(1), s_(1), m(2), s_(2), m(3), s_(3), gain_str);
    end
    fprintf('\n');
end
