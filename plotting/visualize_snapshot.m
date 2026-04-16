%% visualize_snapshot.m — Proposed Scheme Network State at a Specific Round
%
% Replays the proposed scheme loop up to a target round, then draws a
% detailed 2D map showing:
%   - Cluster assignments (member→CH lines, colored per cluster)
%   - CH positions (star markers)
%   - Node jamming risk (node fill color: white=JR=0, red=JR=1)
%   - UAV jammer position + jamming zone
%   - UAV circular orbit path
%   - Inter-cluster routing paths (Dijkstra result, thick lines with arrows)
%   - Dead nodes (gray ×)
%   - Base Station
%
% Usage:
%   Set snapshot_round to any integer (e.g. 200).
%   Set snapshot_round = 0 to pick a random round in [75, 400].

clc; clear; close all;
addpath(genpath('.'));

%% ---- User Settings -------------------------------------------------------
snapshot_round = 700;    % target round — set to 0 for random pick in [75, 400]
seed           = 7;   % RNG seed (matches main.m)
%% -------------------------------------------------------------------------

rng(seed);
config;
init_network;
uav_trajectory;

if snapshot_round == 0
    snapshot_round = randi([75, 400]);
    fprintf('Random snapshot round selected: %d\n', snapshot_round);
end
snapshot_round = min(snapshot_round, T);   % clamp to simulation length

fprintf('Replaying proposed scheme to round %d...\n', snapshot_round);

%% --- Replay proposed scheme loop -----------------------------------------
energy    = E0 * ones(1, N);
PDR_ewma  = ones(1, N);
JR        = zeros(1, N);
alive     = true(1, N);
is_CH     = false(1, N);
CH_assign = zeros(1, N);
snap_paths = cell(1, N);
ch_died_last_round = false;
M_min = max(1, round(0.2 * M));

for t = 1:snapshot_round

    %% CH election (every K_elec rounds, round 1, or emergency)
    need_regular_election   = (mod(t, K_elec) == 0 || t == 1);
    need_emergency_election = ~need_regular_election && (ch_died_last_round || ~any(is_CH & alive));

    if need_regular_election || need_emergency_election
        [is_CH, CH_assign] = elect_ch_proposed(x, y, alive, energy, JR, ...
            dist_to_BS, E0, d_max, p_CH, r_c, r_exc, alpha, beta, gamma_, delta, r_tx);

        CH_idx_e = find(is_CH);
        for c = CH_idx_e
            members_c = find(CH_assign == c & alive);
            if isempty(members_c); continue; end
            avg_d = mean(sqrt((x(members_c)-x(c)).^2 + (y(members_c)-y(c)).^2));
            energy(c) = energy(c) - compute_energy('overhead', L, E_elec, E_amp, E_da, avg_d, 0);
        end
        for i = find(alive & ~is_CH)
            ch = CH_assign(i);
            if ch == 0; continue; end
            d_to_CH = sqrt((x(i)-x(ch))^2 + (y(i)-y(ch))^2);
            energy(i) = energy(i) - compute_energy('overhead', L, E_elec, E_amp, E_da, d_to_CH, 0);
        end
    end

    %% Packet success + JR update
    p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);
    [PDR_ewma, JR] = update_jamming_risk(p, alive, is_CH, PDR_ewma, JR, M, lambda);
    M_eff = max(M_min, round(M * (1 - JR)));

    %% Member → CH transmission energy (scaled by M_eff/M)
    for i = find(alive & ~is_CH)
        ch = CH_assign(i);
        if ch == 0; continue; end
        scale      = M_eff(i) / M;
        d_to_CH    = sqrt((x(i)-x(ch))^2 + (y(i)-y(ch))^2);
        energy(i)  = energy(i)  - scale * compute_energy('tx', L, E_elec, E_amp, E_da, d_to_CH, 0);
        energy(ch) = energy(ch) - scale * compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
    end

    %% Inter-cluster routing
    [snap_paths, ~] = route_dijkstra(x, y, is_CH, JR, BS, phi1, phi2, phi3, E_amp, L);

    %% CH aggregation + forwarding energy
    CH_idx_r = find(is_CH & alive);
    for c = CH_idx_r
        members_c = find(CH_assign == c & alive);
        n_members = length(members_c);
        if n_members == 0; continue; end
        energy(c) = energy(c) - compute_energy('agg', L, E_elec, E_amp, E_da, 0, n_members);
        path = snap_paths{c};
        if isempty(path); continue; end
        for h = 1:length(path)-1
            ni = path(h); nj = path(h+1);
            if nj == N+1
                d_hop = sqrt((x(ni)-BS(1))^2 + (y(ni)-BS(2))^2);
                energy(ni) = energy(ni) - compute_energy('tx', L, E_elec, E_amp, E_da, d_hop, 0);
            else
                d_hop = sqrt((x(ni)-x(nj))^2 + (y(ni)-y(nj))^2);
                energy(ni) = energy(ni) - compute_energy('tx', L, E_elec, E_amp, E_da, d_hop, 0);
                energy(nj) = energy(nj) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
            end
        end
    end

    %% Node death
    newly_dead = alive & (energy <= 0);
    ch_died_last_round = any(is_CH & newly_dead);
    alive(newly_dead) = false;
    if ~any(alive); break; end
end

n_stranded_snap = sum(alive & ~is_CH & (CH_assign == 0));
fprintf('Done. Alive: %d/%d  |  CHs: %d  |  Stranded: %d\n', ...
    sum(alive), N, sum(is_CH & alive), n_stranded_snap);

%% --- Plot ----------------------------------------------------------------
figure('Position', [80, 80, 960, 860], 'Color', 'w');
hold on; axis equal;

CH_idx   = find(is_CH & alive);
n_CH     = length(CH_idx);
cpal     = lines(max(n_CH, 1));   % one color per cluster
cmap_jr  = hot(256);              % JR heatmap: white→yellow→red

%% 1. UAV orbit (decorative dashed circle)
theta = linspace(0, 2*pi, 300);
plot(area/2 + 35*cos(theta), area/2 + 35*sin(theta), ...
    '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1);

%% 2. Jamming zone (filled semi-transparent red circle)
jx_now = J_x(snapshot_round);
jy_now = J_y(snapshot_round);
jzone_x = jx_now + r_j * cos(theta);
jzone_y = jy_now + r_j * sin(theta);
fill(jzone_x, jzone_y, [1 0.55 0.55], ...
    'EdgeColor', [0.85 0 0], 'LineWidth', 1.8, ...
    'FaceAlpha', 0.22, 'LineStyle', '--');

%% 3. Member → CH lines (thin, cluster color, slightly transparent)
for ci = 1:n_CH
    c   = CH_idx(ci);
    col = cpal(ci, :);
    mem = find(CH_assign == c & alive & ~is_CH);
    for i = mem
        plot([x(i), x(c)], [y(i), y(c)], ...
            '-', 'Color', [col, 0.25], 'LineWidth', 0.9);
    end
end

%% 4. Routing paths (thick, cluster color, with hop markers)
for ci = 1:n_CH
    c    = CH_idx(ci);
    col  = cpal(ci, :);
    path = snap_paths{c};
    if isempty(path); continue; end

    % Build coordinate vectors for path (N+1 → BS)
    px = zeros(1, length(path));
    py = zeros(1, length(path));
    for h = 1:length(path)
        if path(h) == N+1
            px(h) = BS(1); py(h) = BS(2);
        else
            px(h) = x(path(h)); py(h) = y(path(h));
        end
    end

    % Draw path segments with arrow at each hop transition
    for h = 1:length(path)-1
        dx = px(h+1) - px(h);
        dy = py(h+1) - py(h);
        % Line segment
        plot([px(h), px(h+1)], [py(h), py(h+1)], ...
            '-', 'Color', col, 'LineWidth', 2.8);
        % Arrowhead at midpoint
        quiver(px(h) + 0.45*dx, py(h) + 0.45*dy, 0.1*dx, 0.1*dy, 0, ...
            'Color', col, 'LineWidth', 2, 'MaxHeadSize', 4, 'AutoScale', 'off');
    end
end

%% 5. Dead nodes
dead_idx = find(~alive);
if ~isempty(dead_idx)
    plot(x(dead_idx), y(dead_idx), 'x', ...
        'MarkerSize', 9, 'Color', [0.65 0.65 0.65], 'LineWidth', 1.5);
end

%% 5b. Stranded nodes (alive, not CH, no CH within r_tx)
stranded_idx = find(alive & ~is_CH & (CH_assign == 0));
if ~isempty(stranded_idx)
    plot(x(stranded_idx), y(stranded_idx), 'd', ...
        'MarkerSize', 9, 'MarkerFaceColor', [0.85 0.85 0.10], ...
        'MarkerEdgeColor', [0.50 0.50 0], 'LineWidth', 1.4);
end

%% 6. Alive member nodes (face color = JR heat, edge color = cluster)
for ci = 1:n_CH
    c   = CH_idx(ci);
    col = cpal(ci, :);
    mem = find(CH_assign == c & alive & ~is_CH);
    for i = mem
        jr_i  = max(0, min(1, JR(i)));
        fc    = cmap_jr(max(1, round(jr_i * 255) + 1), :);
        plot(x(i), y(i), 'o', ...
            'MarkerSize', 9, ...
            'MarkerFaceColor', fc, ...
            'MarkerEdgeColor', col, ...
            'LineWidth', 1.4);
    end
end

%% 7. CH nodes (pentagram, larger, same JR heatmap face)
for ci = 1:n_CH
    c   = CH_idx(ci);
    col = cpal(ci, :);
    jr_c = max(0, min(1, JR(c)));
    fc   = cmap_jr(max(1, round(jr_c * 255) + 1), :);
    plot(x(c), y(c), 'p', ...
        'MarkerSize', 20, ...
        'MarkerFaceColor', fc, ...
        'MarkerEdgeColor', col, ...
        'LineWidth', 2.2);
    text(x(c) + 2, y(c) + 2.5, sprintf('CH%d', ci), ...
        'Color', col, 'FontWeight', 'bold', 'FontSize', 8.5);
end

%% 8. UAV jammer marker
plot(jx_now, jy_now, 'r*', 'MarkerSize', 22, 'LineWidth', 2.5);
text(jx_now + 2, jy_now + 3, 'UAV', ...
    'Color', [0.85 0 0], 'FontWeight', 'bold', 'FontSize', 10);

%% 9. Base Station
plot(BS(1), BS(2), 's', ...
    'MarkerSize', 14, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
text(BS(1) + 2, BS(2) + 2.5, 'BS', ...
    'FontWeight', 'bold', 'FontSize', 10, 'Color', 'k');

%% Colorbar for JR
colormap(cmap_jr);
cb = colorbar('Location', 'eastoutside');
cb.Label.String = 'Jamming Risk JR  (node fill color)';
cb.Label.FontSize = 10;
clim([0 1]);

%% Axes + title
xlim([0 area]); ylim([0 area]);
xlabel('x (m)', 'FontSize', 11);
ylabel('y (m)', 'FontSize', 11);
title(sprintf('Proposed Scheme — Round %d  |  Alive: %d/%d  |  CHs: %d  |  UAV @ (%.1f, %.1f)', ...
    snapshot_round, sum(alive), N, n_CH, jx_now, jy_now), ...
    'FontSize', 12, 'FontWeight', 'bold');
grid on; box on;
set(gca, 'FontSize', 10, 'Layer', 'top');

%% Legend (representative handles)
h_mem      = plot(nan, nan, 'o',  'MarkerSize', 9,  'MarkerFaceColor', [0.9 0.9 0.9], 'MarkerEdgeColor', [0.2 0.4 0.7]);
h_ch       = plot(nan, nan, 'p',  'MarkerSize', 14, 'MarkerFaceColor', [0.9 0.9 0.9], 'MarkerEdgeColor', [0.2 0.4 0.7]);
h_stranded = plot(nan, nan, 'd',  'MarkerSize', 9,  'MarkerFaceColor', [0.85 0.85 0.10], 'MarkerEdgeColor', [0.50 0.50 0]);
h_uav      = plot(nan, nan, 'r*', 'MarkerSize', 12, 'LineWidth', 2);
h_bs       = plot(nan, nan, 's',  'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
h_zone     = fill(nan, nan, [1 0.55 0.55], 'EdgeColor', [0.85 0 0], 'FaceAlpha', 0.22, 'LineStyle', '--');
h_orb      = plot(nan, nan, '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1);
h_path     = plot(nan, nan, '-',  'Color', [0.3 0.3 0.3], 'LineWidth', 2.8);
h_dead     = plot(nan, nan, 'x',  'MarkerSize', 9,  'Color', [0.65 0.65 0.65], 'LineWidth', 1.5);
legend([h_mem, h_ch, h_stranded, h_uav, h_bs, h_zone, h_orb, h_path, h_dead], ...
    {'Member node (fill=JR)', 'Cluster Head ★', sprintf('Stranded (>%dm from CH)', r_tx), ...
     'UAV Jammer', 'Base Station', 'Jamming zone', 'UAV orbit', 'Routing path', 'Dead node'}, ...
    'Location', 'northwest', 'FontSize', 9, 'Box', 'on');

hold off;
fprintf('Plot ready — round %d\n', snapshot_round);
