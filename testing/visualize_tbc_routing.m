%% visualize_tbc_routing.m — TBC Routing Decisions at a Specific Round
%
% Replays the TBC loop up to a target round, then draws a 2D map showing:
%   - Routing paths: colored lines with arrows from each node toward BS
%   - Relay load: node size/color scaled by how many upstream nodes route through it
%   - Jammed nodes (red X) — excluded from routing graph this round
%   - Isolated nodes (yellow diamond) — alive but no path to BS
%   - Dead nodes (gray X)
%   - UAV jammer position + jamming zone
%   - UAV orbit
%   - r_tx radius ring around BS (shows which nodes can reach BS directly)
%   - Base Station
%
% Usage: set snapshot_round and seed below, run from project root.
% Early rounds (1-20) are most interesting — that's where TBC is alive.

clc; clear; close all;
addpath(genpath('.'));

%% ---- User Settings -------------------------------------------------------
snapshot_round = 50;   % target round (TBC dies ~round 18, so 1-17 are meaningful)
seed           = 1;    % RNG seed
%% -------------------------------------------------------------------------

rng(seed);
config;
init_network;
uav_trajectory;

snapshot_round = min(snapshot_round, T);
fprintf('Replaying TBC to round %d...\n', snapshot_round);

%% --- Replay TBC loop to target round -------------------------------------
N_          = N;
T_threshold = 0.5;
BS_idx      = N_ + 1;

energy = E0 * ones(1, N_);
alive  = true(1, N_);

% Precompute distances
dist_nodes = zeros(N_, N_);
for i = 1:N_
    for j = i+1:N_
        d = sqrt((x(i)-x(j))^2 + (y(i)-y(j))^2);
        dist_nodes(i,j) = d;
        dist_nodes(j,i) = d;
    end
end
dist_to_BS = sqrt((x - BS(1)).^2 + (y - BS(2)).^2);

% State captured at snapshot round
snap_jammed   = false(1, N_);
snap_next_hop = zeros(1, N_+1);
snap_D        = inf * ones(1, N_+1);
snap_relay_load = zeros(1, N_);   % how many source nodes route through each node

for t = 1:snapshot_round

    p   = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);
    T_i = zeros(1, N_);
    for i = find(alive)
        T_i(i) = sum(rand(M, 1) <= p(i)) / M;
    end
    jammed   = alive & (T_i < T_threshold);
    routable = alive & ~jammed;

    %% Dijkstra from BS
    inf_dist  = 1e9;
    D         = inf_dist * ones(1, N_+1);
    visited   = false(1, N_+1);
    next_hop  = zeros(1, N_+1);
    D(BS_idx) = 0;

    for iter = 1:N_+1
        D_copy = D; D_copy(visited) = inf_dist;
        [~, u] = min(D_copy);
        if D(u) == inf_dist; break; end
        visited(u) = true;

        if u == BS_idx
            for v = find(routable)
                if dist_to_BS(v) <= r_tx
                    new_d = dist_to_BS(v);
                    if new_d < D(v)
                        D(v) = new_d;
                        next_hop(v) = BS_idx;
                    end
                end
            end
        else
            for v = find(routable)
                if v == u; continue; end
                if dist_nodes(u,v) <= r_tx
                    new_d = D(u) + dist_nodes(u,v);
                    if new_d < D(v)
                        D(v) = new_d;
                        next_hop(v) = u;
                    end
                end
            end
            if dist_to_BS(u) <= r_tx
                new_d = D(u) + dist_to_BS(u);
                if new_d < D(BS_idx)
                    D(BS_idx) = new_d;
                    next_hop(BS_idx) = u;
                end
            end
        end
    end

    %% Compute relay load: count how many sources route through each node
    relay_load = zeros(1, N_);
    for i = find(alive & ~jammed)
        if D(i) == inf_dist; continue; end
        cur = i;
        hops = 0;
        while cur ~= BS_idx && hops <= N_
            nh = next_hop(cur);
            if nh == 0; break; end
            if nh ~= BS_idx
                relay_load(nh) = relay_load(nh) + 1;
            end
            cur  = nh;
            hops = hops + 1;
        end
    end

    %% Energy deduction (same as run_tbc.m)
    energy_delta = zeros(1, N_);
    for i = find(alive)
        if jammed(i); continue; end
        if D(i) == inf_dist; continue; end
        recv_packets = M;
        cur  = i;
        hops = 0;
        while cur ~= BS_idx
            nh = next_hop(cur);
            if nh == 0; break; end
            if nh == BS_idx
                d_hop = dist_to_BS(cur);
            else
                d_hop = dist_nodes(cur, nh);
            end
            energy_delta(cur) = energy_delta(cur) - recv_packets*(L*E_elec + L*E_amp*d_hop^2);
            if nh ~= BS_idx
                energy_delta(nh) = energy_delta(nh) - recv_packets*L*E_elec;
            end
            for pk = 1:recv_packets
                if rand() > p(cur); recv_packets = recv_packets - 1; end
            end
            cur  = nh;
            hops = hops + 1;
            if hops > N_; break; end
        end
    end
    energy = energy + energy_delta;

    %% Capture snapshot state at the target round
    if t == snapshot_round
        snap_jammed     = jammed;
        snap_next_hop   = next_hop;
        snap_D          = D;
        snap_relay_load = relay_load;
        snap_p          = p;
    end

    %% Node death
    newly_dead = alive & (energy <= 0);
    alive(newly_dead) = false;
    if ~any(alive); break; end
end

fprintf('Done. Alive: %d/%d  |  Jammed: %d  |  Isolated: %d  |  Dead: %d\n', ...
    sum(alive), N_, sum(snap_jammed), ...
    sum(alive & ~snap_jammed & (snap_D(1:N_) == 1e9)), ...
    sum(~alive));

%% --- Plot ----------------------------------------------------------------
figure('Position', [80, 80, 960, 860], 'Color', 'w');
hold on; axis equal;

theta = linspace(0, 2*pi, 300);

%% 1. UAV orbit
plot(area/2 + 35*cos(theta), area/2 + 35*sin(theta), ...
    '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1);

%% 2. r_tx ring around BS (nodes inside can reach BS in one hop)
plot(BS(1) + r_tx*cos(theta), BS(2) + r_tx*sin(theta), ...
    ':', 'Color', [0.4 0.6 1.0], 'LineWidth', 1.4);

%% 3. Jamming zone
jx_now = J_x(snapshot_round);
jy_now = J_y(snapshot_round);
fill(jx_now + r_j*cos(theta), jy_now + r_j*sin(theta), ...
    [1 0.55 0.55], 'EdgeColor', [0.85 0 0], 'LineWidth', 1.8, ...
    'FaceAlpha', 0.22, 'LineStyle', '--');

%% 4. Routing paths — one color per path, arrows at midpoint of each hop
% Use a fixed colormap entry per source for visual clarity (not cluster-based).
% Color encodes source node index so you can trace individual paths.
cpal = lines(N_);

for i = find(alive & ~snap_jammed)
    if snap_D(i) == 1e9; continue; end   % isolated — no path
    col = cpal(i, :);
    cur = i;
    hops = 0;
    while cur ~= BS_idx && hops <= N_
        nh = snap_next_hop(cur);
        if nh == 0; break; end
        if nh == BS_idx
            x2 = BS(1); y2 = BS(2);
        else
            x2 = x(nh); y2 = y(nh);
        end
        x1 = x(cur); y1 = y(cur);
        dx = x2 - x1; dy = y2 - y1;
        plot([x1 x2], [y1 y2], '-', 'Color', [col 0.35], 'LineWidth', 1.2);
        % Arrow at 45% along segment
        quiver(x1+0.45*dx, y1+0.45*dy, 0.1*dx, 0.1*dy, 0, ...
            'Color', col, 'LineWidth', 1.2, 'MaxHeadSize', 5, 'AutoScale', 'off');
        cur  = nh;
        hops = hops + 1;
    end
end

%% 5. Dead nodes
dead_idx = find(~alive);
if ~isempty(dead_idx)
    plot(x(dead_idx), y(dead_idx), 'x', ...
        'MarkerSize', 9, 'Color', [0.65 0.65 0.65], 'LineWidth', 1.5);
end

%% 6. Isolated nodes (alive, not jammed, but no path to BS)
iso_idx = find(alive & ~snap_jammed & (snap_D(1:N_) == 1e9));
if ~isempty(iso_idx)
    plot(x(iso_idx), y(iso_idx), 'd', ...
        'MarkerSize', 10, 'MarkerFaceColor', [0.95 0.85 0.1], ...
        'MarkerEdgeColor', [0.6 0.5 0], 'LineWidth', 1.4);
end

%% 7. Jammed nodes (red X, larger)
jam_idx = find(snap_jammed);
if ~isempty(jam_idx)
    plot(x(jam_idx), y(jam_idx), 'x', ...
        'MarkerSize', 13, 'Color', [0.85 0 0], 'LineWidth', 2.2);
end

%% 8. Routable nodes — size and face color encode relay load
%    Small dot = originates only (relay_load=0)
%    Larger + darker = heavy relay burden
max_load = max(snap_relay_load) + 1;
for i = find(alive & ~snap_jammed)
    load_i    = snap_relay_load(i);
    norm_load = load_i / max_load;           % 0 = no relay, 1 = max relay
    sz        = 8 + 16 * norm_load;         % marker size 8 to 24
    % Color: light blue (no relay) → dark orange (heavy relay)
    fc = (1 - norm_load) * [0.55 0.78 1.0] + norm_load * [1.0 0.45 0.1];
    plot(x(i), y(i), 'o', ...
        'MarkerSize', sz, ...
        'MarkerFaceColor', fc, ...
        'MarkerEdgeColor', [0.2 0.2 0.2], ...
        'LineWidth', 1.0);
    % Label relay count if > 0
    if load_i > 0
        text(x(i)+1.5, y(i)+1.5, num2str(load_i), ...
            'FontSize', 7, 'Color', [0.15 0.15 0.15]);
    end
end

%% 9. UAV jammer
plot(jx_now, jy_now, 'r*', 'MarkerSize', 22, 'LineWidth', 2.5);
text(jx_now+2, jy_now+3, 'UAV', 'Color', [0.85 0 0], ...
    'FontWeight', 'bold', 'FontSize', 10);

%% 10. Base Station
plot(BS(1), BS(2), 's', 'MarkerSize', 16, ...
    'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
text(BS(1)+2, BS(2)+2.5, 'BS', ...
    'FontWeight', 'bold', 'FontSize', 10, 'Color', 'k');

%% Axes + title
xlim([0 area]); ylim([0 area]);
xlabel('x (m)', 'FontSize', 11);
ylabel('y (m)', 'FontSize', 11);
title(sprintf('TBC Routing — Round %d  |  Alive: %d/%d  |  Jammed: %d  |  Isolated: %d', ...
    snapshot_round, sum(alive), N_, sum(snap_jammed), length(iso_idx)), ...
    'FontSize', 12, 'FontWeight', 'bold');
grid on; box on;
set(gca, 'FontSize', 10, 'Layer', 'top');

%% Legend
h_route  = plot(nan, nan, '-',  'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
h_node0  = plot(nan, nan, 'o',  'MarkerSize', 8,  'MarkerFaceColor', [0.55 0.78 1.0], 'MarkerEdgeColor', [0.2 0.2 0.2]);
h_nodeh  = plot(nan, nan, 'o',  'MarkerSize', 18, 'MarkerFaceColor', [1.0 0.45 0.1],  'MarkerEdgeColor', [0.2 0.2 0.2]);
h_jam    = plot(nan, nan, 'x',  'MarkerSize', 13, 'Color', [0.85 0 0], 'LineWidth', 2.2);
h_iso    = plot(nan, nan, 'd',  'MarkerSize', 10, 'MarkerFaceColor', [0.95 0.85 0.1], 'MarkerEdgeColor', [0.6 0.5 0]);
h_dead   = plot(nan, nan, 'x',  'MarkerSize', 9,  'Color', [0.65 0.65 0.65], 'LineWidth', 1.5);
h_uav    = plot(nan, nan, 'r*', 'MarkerSize', 12, 'LineWidth', 2);
h_bs     = plot(nan, nan, 's',  'MarkerSize', 10, 'MarkerFaceColor', 'k');
h_zone   = fill(nan, nan, [1 0.55 0.55], 'EdgeColor', [0.85 0 0], 'FaceAlpha', 0.22, 'LineStyle', '--');
h_rtx    = plot(nan, nan, ':',  'Color', [0.4 0.6 1.0], 'LineWidth', 1.4);
h_orb    = plot(nan, nan, '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1);
legend([h_route, h_node0, h_nodeh, h_jam, h_iso, h_dead, h_uav, h_bs, h_zone, h_rtx, h_orb], ...
    {'Routing path', 'Node (relay load=0)', 'Node (heavy relay, label=count)', ...
     'Jammed (suppressed)', 'Isolated (no path)', 'Dead', ...
     'UAV Jammer', 'Base Station', 'Jamming zone', ...
     sprintf('r_{tx}=%.0fm ring (1-hop to BS)', r_tx), 'UAV orbit'}, ...
    'Location', 'northwest', 'FontSize', 8.5, 'Box', 'on');

hold off;
fprintf('Plot ready — round %d\n', snapshot_round);
