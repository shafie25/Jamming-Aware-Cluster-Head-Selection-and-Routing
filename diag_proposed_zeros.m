%% diag_proposed_zeros.m — Diagnostic: Zero-PDR rounds in the Proposed Scheme
% Runs the proposed scheme for seed 42 and prints every zero-PDR round with cause info.
clc; clear;
addpath(genpath('.'));

rng(42);
config;
init_network;
uav_trajectory;

N = length(x);

energy    = E0 * ones(1, N);
PDR_ewma  = ones(1, N);
JR        = zeros(1, N);
alive     = true(1, N);
is_CH     = false(1, N);
CH_assign = zeros(1, N);

PDR_per_round = zeros(1, T);

fprintf('Zero-PDR rounds for Proposed Scheme (seed 42, kappa=%.0f):\n', kappa);
fprintf('%-6s | %-6s | %-4s | %-8s | %-7s | %-10s | %-9s | %s\n', ...
    'Round','Alive','CHs','Stranded','Members','avg_p_mem','CHs_jammed','Cause');
fprintf('%s\n', repmat('-', 1, 85));

for t = 1:T

    %% CH Election (every K_elec rounds)
    if mod(t, K_elec) == 0 || t == 1
        [is_CH, CH_assign] = elect_ch_proposed(x, y, alive, energy, JR, ...
            dist_to_BS, E0, d_max, r_c, r_exc, alpha, beta, gamma_, delta, r_tx);

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

    %% Packet Success + JR Update
    p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);
    [PDR_ewma, JR] = update_jamming_risk(p, alive, is_CH, PDR_ewma, JR, M, lambda);

    %% Member Transmission
    for i = find(alive & ~is_CH)
        ch = CH_assign(i);
        if ch == 0; continue; end
        d_to_CH    = sqrt((x(i)-x(ch))^2 + (y(i)-y(ch))^2);
        energy(i)  = energy(i)  - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_CH, 0);
        energy(ch) = energy(ch) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
    end

    %% CH Aggregation + Routing
    [paths, hop_counts] = route_dijkstra(x, y, is_CH, JR, BS, phi1, phi2, phi3, E_amp, L);

    total_recv = 0; total_sent = 0; total_delay = 0; n_CH_active = 0;
    CH_idx = find(is_CH & alive);

    for c = CH_idx
        members_c = find(CH_assign == c & alive);
        n_members = length(members_c);
        if n_members == 0; continue; end

        energy(c) = energy(c) - compute_energy('agg', L, E_elec, E_amp, E_da, 0, n_members);
        p_members  = p(members_c);
        recv_count = sum(rand(M, n_members) <= p_members, 'all');
        total_recv = total_recv + recv_count;
        total_sent = total_sent + n_members * M;

        path = paths{c};
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
        total_delay = total_delay + hop_counts(c);
        n_CH_active = n_CH_active + 1;
    end

    n_stranded = sum(alive & ~is_CH & (CH_assign == 0));
    total_sent = total_sent + n_stranded * M;

    PDR_per_round(t) = (total_sent > 0) * total_recv / max(total_sent, 1);

    %% Log zero-PDR rounds
    if PDR_per_round(t) == 0
        n_alive_t = sum(alive);
        n_CH_t    = length(CH_idx);
        n_mem_t   = sum(alive & ~is_CH & CH_assign > 0);

        active_mem = find(alive & ~is_CH & CH_assign > 0);
        avg_p = 0;
        if ~isempty(active_mem); avg_p = mean(p(active_mem)); end

        ch_d_jam = sqrt((x(CH_idx)-J_x(t)).^2 + (y(CH_idx)-J_y(t)).^2);
        n_ch_jam = sum(ch_d_jam <= r_j);

        % Classify cause
        if total_sent == 0 && n_alive_t == n_CH_t
            cause = 'ALL_NODES_ARE_CH';
        elseif total_sent == 0 && n_stranded == n_alive_t - n_CH_t
            cause = 'ALL_STRANDED';
        elseif total_sent == 0
            cause = 'NO_SENT(other)';
        elseif avg_p < 0.05
            cause = 'ALL_JAMMED';
        elseif n_mem_t <= 2
            cause = sprintf('STOCHASTIC(mem=%d,p=%.2f)', n_mem_t, avg_p);
        else
            cause = sprintf('STOCHASTIC(mem=%d,p=%.2f)', n_mem_t, avg_p);
        end

        fprintf('%-6d | %-6d | %-4d | %-8d | %-7d | %-10.4f | %-9d | %s\n', ...
            t, n_alive_t, n_CH_t, n_stranded, n_mem_t, avg_p, n_ch_jam, cause);
    end

    %% Node Death
    newly_dead = alive & (energy <= 0);
    alive(newly_dead) = false;
    if ~any(alive); break; end
end

fprintf('%s\n', repmat('-', 1, 85));
fprintf('Total zero-PDR rounds: %d\n', sum(PDR_per_round == 0));

zero_rounds = find(PDR_per_round == 0);
if ~isempty(zero_rounds)
    fprintf('\nZero-PDR round distribution:\n');
    fprintf('  Early   (rounds 1-300):   %d\n', sum(zero_rounds <= 300));
    fprintf('  Middle  (rounds 301-600): %d\n', sum(zero_rounds > 300 & zero_rounds <= 600));
    fprintf('  Late    (rounds 601+):    %d\n', sum(zero_rounds > 600));
end
