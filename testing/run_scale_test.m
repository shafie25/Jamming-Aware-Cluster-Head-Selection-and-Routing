%% run_scale_test.m — Scaled Deployment: 200x200m, 200 Nodes
% Tests whether Dijkstra inter-cluster routing becomes beneficial when the
% network is large enough that most CHs cannot reach the BS directly.
%
% Motivation: in the 100x100m field with BS at center and r_tx=50m, ~78%
% of the field is within direct BS range — Dijkstra degenerates to direct
% for almost every CH. Scaling to 200x200m drops this to ~20%, forcing
% genuine multi-hop inter-cluster routing for ~80% of CHs.
%
% Scale changes vs canonical config:
%   area         100  -> 200  m
%   N            100  -> 200  nodes   (same density: 0.01 nodes/m^2)
%   BS        [50,50] -> [100,100]    (field center)
%   orbit_radius  35  ->  70  m       (same relative orbit: 35% of half-field)
%   r_exc         25  ->  35  m       (sqrt(area^2/(p_CH*N*pi)) for ~10 CHs)
%   r_tx          50  ->  75  m       (scaled up so members can reach CHs; ~44% of field in direct BS range)
%   r_j, kappa, lambda, phi*, E_* all unchanged
%
% Comparison: Proposed (Dijkstra) vs Proposed (Direct) vs LEACH
% Run from project root: >> run('testing/run_scale_test.m')

clc; clear; close all;
addpath(genpath('.'));

seeds   = 1:20;
n_seeds = length(seeds);

%% Load canonical config then override scaled parameters
config;

% --- Scale overrides ---
N            = 200;
area         = 200;
BS           = [100, 100];
orbit_radius = 70;               % keeps UAV well inside 200x200m field
r_exc        = 35;               % sqrt(200^2 / (0.05*200*pi)) ~ 35.7m
r_tx         = 75;               % scaled up from 50m: large enough to cover members
                                  % (avg nearest-CH dist ~63m), small enough that
                                  % only ~44% of field has direct BS reach — most
                                  % CHs still need multi-hop routing
% All energy, lambda, kappa, phi*, r_j unchanged

%% Recompute d_max for CHScore normalization (diagonal of new field)
d_max_scaled = sqrt(2) * area;   % ~283m

omega = 2*pi / 50;   % angular speed (rad/round) — same orbit period as canonical

%% Preallocate
schemes = {'dijkstra', 'direct', 'leach'};
labels  = {'Proposed (Dijkstra)', 'Proposed (Direct)', 'LEACH'};
n_sch   = length(schemes);

store = struct();
for k = 1:n_sch
    s = schemes{k};
    store.PDR.(s)    = zeros(n_seeds, T);
    store.energy.(s) = zeros(n_seeds, T);
    store.alive.(s)  = zeros(n_seeds, T);
    store.tdeath.(s) = NaN(n_seeds, 1);
end

%% Main loop
for si = 1:n_seeds
    fprintf('=== Seed %d (%d/%d) ===\n', seeds(si), si, n_seeds);

    %% Proposed + Dijkstra
    rng(seeds(si));
    x = rand(1, N) * area;
    y = rand(1, N) * area;
    dist_to_BS = sqrt((x - BS(1)).^2 + (y - BS(2)).^2);
    t_vec = 1:T;
    J_x = BS(1) + orbit_radius * cos(omega * t_vec);
    J_y = BS(2) + orbit_radius * sin(omega * t_vec);

    r = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
        E0, d_max_scaled, T, K_elec, M, lambda, p_CH, r_c, r_exc, ...
        alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
    store.PDR.dijkstra(si,:)    = r.PDR;
    store.energy.dijkstra(si,:) = r.energy;
    store.alive.dijkstra(si,:)  = r.alive;
    store.tdeath.dijkstra(si)   = r.t_death;
    fprintf('  Dijkstra  t_death=%d  PDR_all=%.2f%%  ZeroPDR=%d\n', ...
        r.t_death, mean(r.PDR)*100, sum(r.PDR==0));

    %% Proposed + Direct
    rng(seeds(si));
    x = rand(1, N) * area;
    y = rand(1, N) * area;
    dist_to_BS = sqrt((x - BS(1)).^2 + (y - BS(2)).^2);
    J_x = BS(1) + orbit_radius * cos(omega * t_vec);
    J_y = BS(2) + orbit_radius * sin(omega * t_vec);

    r = run_proposed_direct(x, y, BS, J_x, J_y, dist_to_BS, ...
        E0, d_max_scaled, T, K_elec, M, lambda, p_CH, r_c, r_exc, ...
        alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
    store.PDR.direct(si,:)    = r.PDR;
    store.energy.direct(si,:) = r.energy;
    store.alive.direct(si,:)  = r.alive;
    store.tdeath.direct(si)   = r.t_death;
    fprintf('  Direct    t_death=%d  PDR_all=%.2f%%  ZeroPDR=%d\n', ...
        r.t_death, mean(r.PDR)*100, sum(r.PDR==0));

    %% LEACH
    rng(seeds(si));
    x = rand(1, N) * area;
    y = rand(1, N) * area;
    J_x = BS(1) + orbit_radius * cos(omega * t_vec);
    J_y = BS(2) + orbit_radius * sin(omega * t_vec);

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
fprintf('\n\n================================================================\n');
fprintf('Scale Test — 200x200m, 200 nodes, BS=[100,100], r_tx=%dm\n', r_tx);
fprintf('(~%.0f%% of field within direct BS range vs 78%% in canonical)\n', min(100, pi*r_tx^2/area^2*100));
fprintf('================================================================\n\n');

col_w = 26;
fprintf('%-22s', 'Metric');
for k = 1:n_sch; fprintf(' | %-*s', col_w, labels{k}); end
fprintf('\n%s\n', repmat('-', 1, 22 + n_sch*(col_w+3)));

metric_labels = {'First death (rnd)', 'PDR all rounds (%)', ...
                 'PDR FND-trunc (%)', 'Zero-PDR rounds', 'Energy@r300 (J)'};
metric_keys   = {'tdeath','all','fnd','zero','r300'};

for mi = 1:length(metric_keys)
    fprintf('%-22s', metric_labels{mi});
    vals = zeros(n_seeds, n_sch);
    for k = 1:n_sch
        fd = schemes{k};
        switch metric_keys{mi}
            case 'tdeath'
                vals(:,k) = store.tdeath.(fd);
            case 'all'
                vals(:,k) = mean(store.PDR.(fd), 2) * 100;
            case 'fnd'
                for s = 1:n_seeds
                    td = store.tdeath.(fd)(s);
                    if ~isnan(td) && td >= 1
                        vals(s,k) = mean(store.PDR.(fd)(s,1:round(td)))*100;
                    else
                        vals(s,k) = mean(store.PDR.(fd)(s,:))*100;
                    end
                end
            case 'zero'
                vals(:,k) = sum(store.PDR.(fd) == 0, 2);
            case 'r300'
                vals(:,k) = store.energy.(fd)(:, 300);
        end
        m_v = mean(vals(:,k), 'omitnan');
        s_v = std(vals(:,k),  'omitnan');
        fprintf(' | %6.1f +/- %5.1f        ', m_v, s_v);
    end
    fprintf('\n');
end
fprintf('%s\n', repmat('-', 1, 22 + n_sch*(col_w+3)));

%% Routing gain summary
fprintf('\nRouting gain (Dijkstra - Direct):\n');
for mi = 1:length(metric_keys)
    fd_d = schemes{1}; fd_r = schemes{2};
    switch metric_keys{mi}
        case 'tdeath'
            v_d = store.tdeath.(fd_d); v_r = store.tdeath.(fd_r);
        case 'all'
            v_d = mean(store.PDR.(fd_d),2)*100; v_r = mean(store.PDR.(fd_r),2)*100;
        case 'fnd'
            v_d = zeros(n_seeds,1); v_r = zeros(n_seeds,1);
            for s = 1:n_seeds
                for ki = 1:2
                    fd = schemes{ki};
                    td = store.tdeath.(fd)(s);
                    vv = (td>=1 & ~isnan(td)) * mean(store.PDR.(fd)(s,1:max(1,round(td))))*100 ...
                       + (isnan(td)|td<1) * mean(store.PDR.(fd)(s,:))*100;
                    if ki==1; v_d(s)=vv; else; v_r(s)=vv; end
                end
            end
        case 'zero'
            v_d = sum(store.PDR.(fd_d)==0,2); v_r = sum(store.PDR.(fd_r)==0,2);
        case 'r300'
            v_d = store.energy.(fd_d)(:,300); v_r = store.energy.(fd_r)(:,300);
    end
    gain = mean(v_d,'omitnan') - mean(v_r,'omitnan');
    fprintf('  %-22s  %+.2f\n', metric_labels{mi}, gain);
end

fprintf('\nDone. Direct BS coverage ~%.0f%% of 200x200m field at r_tx=%dm.\n', ...
    min(100, pi*r_tx^2/area^2*100), r_tx);
