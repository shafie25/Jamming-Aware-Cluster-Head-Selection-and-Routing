%% run_reactive_ch.m — Baseline 3: Reactive CH Re-election on Jamming Detection
% Models the FCPA (Sensors/Springer 2023) simplified approach: reactive
% re-clustering triggered when a CH is detected as jammed. Standard LEACH
% elects CHs first; any CH whose JR exceeds a threshold is then replaced
% by the best-scoring member in its cluster (highest energy, lowest JR).
%
% Design rationale:
%   - JR_CH_thresh = 0.5 means PDR_ewma < 0.50 — CH delivering fewer than
%     half its packets is considered compromised and replaced
%   - Replacement scoring: energy(i)/E0 - JR(i) — prioritizes energy-rich,
%     low-jamming members (same philosophy as CHScore but simplified, reactive)
%   - All former CH's members are re-assigned to the new CH
%   - Routing remains direct CH→BS (no inter-cluster Dijkstra)
%
% Key distinction from proposed scheme:
%   - Reactive (responds after jammer hits) vs proactive (avoids jammer preemptively)
%   - No JR-aware inter-cluster routing layer
%   - Re-election is triggered per-CH on failure, not recalculated globally

function results = run_reactive_ch(x, y, BS, J_x, J_y, E0, T, M, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L, lambda)

    N            = length(x);
    P            = 0.05;
    JR_CH_thresh = 0.5;   % replace CH if JR > this (PDR_ewma < 0.50)

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

        %% LEACH CH Election (initial election — may be overridden below)
        T_thresh_val = zeros(1, N);
        denom = 1 - P * mod(t - 1, round(1/P));
        if denom <= 0; denom = 1e-6; end
        T_thresh_val(G & alive) = P / denom;
        is_CH = (rand(1, N) < T_thresh_val) & alive;
        if ~any(is_CH); is_CH = alive; end
        G(is_CH) = false;
        if mod(t, round(1/P)) == 0; G = alive; end

        %% Initial Cluster Assignment
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

        %% Reactive CH Re-election
        % For each jammed CH (JR > threshold), find best replacement from its members.
        % Iterate over snapshot of original CHs (is_CH may change mid-loop).
        original_CHs = find(is_CH & alive);
        for c = original_CHs
            if JR(c) <= JR_CH_thresh; continue; end   % CH is fine — no action

            % Find current non-CH members of this cluster
            members_c = find(CH_assign == c & alive & ~is_CH);
            if isempty(members_c); continue; end

            % Score members: reward high energy, penalize high JR
            scores = energy(members_c) / E0 - JR(members_c);
            [~, best_idx] = max(scores);
            new_ch = members_c(best_idx);

            % Promote new CH
            is_CH(new_ch)    = true;
            CH_assign(new_ch) = 0;   % new CH has no parent

            % Demote old CH to member
            is_CH(c)    = false;
            CH_assign(c) = new_ch;

            % Re-assign all remaining members to new CH
            for j = members_c
                if j == new_ch; continue; end
                CH_assign(j) = new_ch;
            end
        end

        %% Member Transmission (all active members transmit)
        CH_idx = find(is_CH & alive);   % refresh after re-election
        for i = find(alive & ~is_CH)
            ch = CH_assign(i);
            if ch == 0; continue; end
            d_to_CH   = sqrt((x(i) - x(ch))^2 + (y(i) - y(ch))^2);
            energy(i)  = energy(i)  - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_CH, 0);
            energy(ch) = energy(ch) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
        end

        %% CH Aggregation + Direct BS Transmission
        total_recv = 0; total_sent = 0; n_CH_active = 0;
        for c = CH_idx
            members_c = find(CH_assign == c & alive);
            n_members = length(members_c);
            if n_members == 0; continue; end
            energy(c) = energy(c) - compute_energy('agg', L, E_elec, E_amp, E_da, 0, n_members);
            p_members  = p(members_c);
            recv_count = sum(rand(M, n_members) <= p_members, 'all');
            total_recv = total_recv + recv_count;
            total_sent = total_sent + n_members * M;
            d_to_BS   = sqrt((x(c) - BS(1))^2 + (y(c) - BS(2))^2);
            energy(c) = energy(c) - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_BS, 0);
            n_CH_active = n_CH_active + 1;
        end

        %% Record Metrics
        PDR_per_round(t)    = (total_sent > 0) * total_recv / max(total_sent, 1);
        energy_per_round(t) = sum(energy(alive));
        delay_per_round(t)  = 1;   % always direct CH→BS
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
    results.label   = 'Reactive-CH';
end
