%% run_threshold.m — Baseline 2: Threshold-Based Member Suppression
% Models IEEE MRIE 2025 approach: PDR threshold with topological node removal.
% Nodes track JR via EWMA. Members whose JR exceeds a threshold are suppressed
% (skip transmission that round) to avoid wasting energy and CH bandwidth on
% heavily-jammed links. CH election is standard LEACH. Routing is direct CH→BS.
%
% Design rationale:
%   - JR_thresh = 0.7 means PDR_ewma < 0.30 — only very badly jammed nodes suppressed
%   - Suppressed nodes save Tx energy; CH saves Rx and aggregation energy for them
%   - Suppressed nodes' M packets count in total_sent denominator (data is lost —
%     honest accounting for fair PDR comparison across all schemes)
%   - This is a reactive, per-node approach with no CH-level adaptation
%
% Differences from EWMA-Detect:
%   + Jammed members are suppressed (don't transmit) — first level of adaptation

function results = run_threshold(x, y, BS, J_x, J_y, E0, T, M, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L, lambda)

    N         = length(x);
    P         = 0.05;
    JR_thresh = 0.7;   % suppress members with JR > this (PDR_ewma < 0.30)

    %% Initialize node state
    energy   = E0 * ones(1, N);
    alive    = true(1, N);
    G        = true(1, N);
    PDR_ewma = ones(1, N);
    JR       = zeros(1, N);

    %% Initialize metrics
    PDR_per_round    = zeros(1, T);
    energy_per_round = zeros(1, T);
    delay_per_round  = zeros(1, T);
    alive_per_round  = zeros(1, T);
    t_death          = NaN;

    for t = 1:T

        %% LEACH CH Election (unchanged)
        T_thresh_val = zeros(1, N);
        denom = 1 - P * mod(t - 1, round(1/P));
        if denom <= 0; denom = 1e-6; end
        T_thresh_val(G & alive) = P / denom;
        is_CH = (rand(1, N) < T_thresh_val) & alive;
        if ~any(is_CH); is_CH = alive; end
        G(is_CH) = false;
        if mod(t, round(1/P)) == 0; G = alive; end

        %% Cluster Assignment
        CH_idx = find(is_CH);
        CH_assign = zeros(1, N);
        for i = find(alive & ~is_CH)
            d_to_CHs = sqrt((x(i) - x(CH_idx)).^2 + (y(i) - y(CH_idx)).^2);
            [~, nearest] = min(d_to_CHs);
            CH_assign(i) = CH_idx(nearest);
        end

        %% Packet Success Probability
        p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);

        %% EWMA JR Update
        [PDR_ewma, JR] = update_jamming_risk(p, alive, is_CH, PDR_ewma, JR, M, lambda);

        %% Member Transmission with Threshold Suppression
        % Suppressed nodes skip Tx (save energy). Their packets are lost.
        for i = find(alive & ~is_CH)
            ch = CH_assign(i);
            if ch == 0; continue; end
            if JR(i) > JR_thresh; continue; end   % suppressed — skip Tx, no energy cost
            d_to_CH   = sqrt((x(i) - x(ch))^2 + (y(i) - y(ch))^2);
            energy(i)  = energy(i)  - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_CH, 0);
            energy(ch) = energy(ch) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
        end

        %% CH Aggregation + PDR Accounting + Direct BS Transmission
        total_recv = 0; total_sent = 0; n_CH_active = 0;
        for c = CH_idx
            all_members    = find(CH_assign == c & alive);
            active_members = find(CH_assign == c & alive & JR <= JR_thresh);
            n_all    = length(all_members);
            n_active = length(active_members);
            if n_all == 0; continue; end

            % All members' M packets count in denominator (suppressed ones deliver 0)
            total_sent = total_sent + n_all * M;

            if n_active > 0
                % CH pays agg and Tx only if active members exist
                energy(c) = energy(c) - compute_energy('agg', L, E_elec, E_amp, E_da, 0, n_active);
                p_active   = p(active_members);
                recv_count = sum(rand(M, n_active) <= p_active, 'all');
                total_recv = total_recv + recv_count;
                d_to_BS   = sqrt((x(c) - BS(1))^2 + (y(c) - BS(2))^2);
                energy(c) = energy(c) - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_BS, 0);
                n_CH_active = n_CH_active + 1;
            end
        end

        %% Record Metrics
        PDR_per_round(t)    = (total_sent > 0) * total_recv / max(total_sent, 1);
        energy_per_round(t) = sum(energy(alive));
        delay_per_round(t)  = 1;
        alive_per_round(t)  = sum(alive);

        %% Node Death
        newly_dead = alive & (energy <= 0);
        if any(newly_dead) && isnan(t_death); t_death = t; end
        alive(newly_dead) = false;
        G(newly_dead) = false;
        if ~any(alive); break; end
    end

    results.PDR     = PDR_per_round;
    results.energy  = energy_per_round;
    results.delay   = delay_per_round;
    results.alive   = alive_per_round;
    results.t_death = t_death;
    results.label   = 'Threshold-JR';
end
