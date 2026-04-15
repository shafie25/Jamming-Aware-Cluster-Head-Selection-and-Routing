%% diag_leach_zeros.m — Diagnostic: Why does LEACH have zero-PDR rounds?
% Runs LEACH for seed 42 and prints every zero-PDR round with cause info.
clc; clear;
addpath(genpath('.'));

rng(42);
config;
init_network;
uav_trajectory;

N   = length(x);
P   = 0.05;

energy = E0 * ones(1, N);
alive  = true(1, N);
G      = true(1, N);

PDR_per_round = zeros(1, T);

fprintf('Zero-PDR rounds for LEACH (seed 42, kappa=%.0f):\n', kappa);
fprintf('%-6s | %-6s | %-4s | %-8s | %-7s | %-10s | %-9s | %s\n', ...
    'Round','Alive','CHs','Stranded','Members','avg_p_mem','CHs_jammed','Cause');
fprintf('%s\n', repmat('-', 1, 80));

for t = 1:T

    %% LEACH CH Election
    T_thresh = zeros(1, N);
    denom = 1 - P * mod(t - 1, round(1/P));
    if denom <= 0; denom = 1e-6; end
    T_thresh(G & alive) = P / denom;
    is_CH = (rand(1, N) < T_thresh) & alive;
    if ~any(is_CH); is_CH = alive; end
    G(is_CH) = false;
    if mod(t, round(1/P)) == 0; G = alive; end

    %% Cluster Assignment (with r_tx)
    CH_idx    = find(is_CH);
    CH_assign = zeros(1, N);
    for i = find(alive & ~is_CH)
        d_to_CHs         = sqrt((x(i)-x(CH_idx)).^2 + (y(i)-y(CH_idx)).^2);
        [min_d, nearest] = min(d_to_CHs);
        if min_d <= r_tx
            CH_assign(i) = CH_idx(nearest);
        end
    end

    %% Packet Success
    p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);

    %% Member Transmission
    for i = find(alive & ~is_CH)
        ch = CH_assign(i);
        if ch == 0; continue; end
        d_to_CH    = sqrt((x(i)-x(ch))^2 + (y(i)-y(ch))^2);
        energy(i)  = energy(i)  - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_CH, 0);
        energy(ch) = energy(ch) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
    end

    %% CH Aggregation + Direct BS Tx
    total_recv = 0; total_sent = 0;
    for c = CH_idx
        members_c = find(CH_assign == c & alive);
        n_members = length(members_c);
        if n_members == 0; continue; end
        energy(c) = energy(c) - compute_energy('agg', L, E_elec, E_amp, E_da, 0, n_members);
        p_members  = p(members_c);
        recv_count = sum(rand(M, n_members) <= p_members, 'all');
        total_recv = total_recv + recv_count;
        total_sent = total_sent + n_members * M;
        d_to_BS   = sqrt((x(c)-BS(1))^2 + (y(c)-BS(2))^2);
        energy(c) = energy(c) - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_BS, 0);
    end

    n_stranded = sum(alive & ~is_CH & (CH_assign == 0));
    total_sent = total_sent + n_stranded * M;

    PDR_per_round(t) = (total_sent > 0) * total_recv / max(total_sent, 1);

    %% Log zero-PDR rounds
    if PDR_per_round(t) == 0
        n_alive_t  = sum(alive);
        n_CH_t     = length(CH_idx);
        n_mem_t    = sum(alive & ~is_CH & CH_assign > 0);

        active_mem = find(alive & ~is_CH & CH_assign > 0);
        avg_p = 0;
        if ~isempty(active_mem); avg_p = mean(p(active_mem)); end

        ch_d_jam   = sqrt((x(CH_idx)-J_x(t)).^2 + (y(CH_idx)-J_y(t)).^2);
        n_ch_jam   = sum(ch_d_jam <= r_j);

        % Classify cause
        if total_sent == 0 && n_alive_t == n_CH_t
            cause = 'ALL_NODES_ARE_CH';
        elseif total_sent == 0 && n_stranded == n_alive_t - n_CH_t
            cause = 'ALL_STRANDED';
        elseif avg_p < 0.05
            cause = 'ALL_JAMMED';
        elseif n_mem_t == 0
            cause = 'NO_MEMBERS';
        else
            cause = 'STOCHASTIC';
        end

        fprintf('%-6d | %-6d | %-4d | %-8d | %-7d | %-10.4f | %-9d | %s\n', ...
            t, n_alive_t, n_CH_t, n_stranded, n_mem_t, avg_p, n_ch_jam, cause);
    end

    %% Node Death
    newly_dead = alive & (energy <= 0);
    alive(newly_dead) = false;
    G(newly_dead) = false;
    if ~any(alive); break; end
end

fprintf('%s\n', repmat('-', 1, 80));
fprintf('Total zero-PDR rounds: %d\n', sum(PDR_per_round == 0));

% Summary by cause bucket
rounds_vec = 1:T;
zero_rounds = rounds_vec(PDR_per_round == 0);
if ~isempty(zero_rounds)
    fprintf('\nZero-PDR round distribution:\n');
    fprintf('  Early   (rounds 1-300):   %d\n', sum(zero_rounds <= 300));
    fprintf('  Middle  (rounds 301-600): %d\n', sum(zero_rounds > 300 & zero_rounds <= 600));
    fprintf('  Late    (rounds 601+):    %d\n', sum(zero_rounds > 600));
end
