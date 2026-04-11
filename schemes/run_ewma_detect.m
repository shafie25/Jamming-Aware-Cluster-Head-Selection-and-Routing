%% run_ewma_detect.m — Baseline 1: EWMA Detection Only (no protocol adaptation)
% Models the approach from IEEE ICDSIS 2022: nodes estimate jamming risk via
% EWMA but the protocol does not adapt — CH election and routing are unchanged
% from standard LEACH. This baseline isolates the value of detection vs action:
% knowing you are being jammed provides zero benefit if the protocol ignores it.
%
% Differences from standard LEACH:
%   + EWMA JR tracking runs every round (same as proposed scheme)
%   - JR is never used in CH election or routing decisions
%
% Inputs/Output: identical signature to run_leach, plus lambda for EWMA.

function results = run_ewma_detect(x, y, BS, J_x, J_y, E0, T, M, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L, lambda)

    N = length(x);
    P = 0.05;   % LEACH target CH fraction

    %% Initialize node state
    energy   = E0 * ones(1, N);
    alive    = true(1, N);
    G        = true(1, N);   % LEACH epoch eligibility
    PDR_ewma = ones(1, N);   % EWMA-smoothed PDR (tracked but not used)
    JR       = zeros(1, N);  % jamming risk (tracked but not used)

    %% Initialize metrics
    PDR_per_round    = zeros(1, T);
    energy_per_round = zeros(1, T);
    delay_per_round  = zeros(1, T);
    alive_per_round  = zeros(1, T);
    t_death          = NaN;

    for t = 1:T

        %% LEACH CH Election (unchanged — JR not used)
        T_thresh = zeros(1, N);
        denom = 1 - P * mod(t - 1, round(1/P));
        if denom <= 0; denom = 1e-6; end
        T_thresh(G & alive) = P / denom;
        is_CH = (rand(1, N) < T_thresh) & alive;
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

        %% EWMA JR Update (detection computed — not used in any decision below)
        [PDR_ewma, JR] = update_jamming_risk(p, alive, is_CH, PDR_ewma, JR, M, lambda);

        %% Member Transmission (standard LEACH — no suppression)
        for i = find(alive & ~is_CH)
            ch = CH_assign(i);
            if ch == 0; continue; end
            d_to_CH   = sqrt((x(i) - x(ch))^2 + (y(i) - y(ch))^2);
            energy(i)  = energy(i)  - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_CH, 0);
            energy(ch) = energy(ch) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
        end

        %% CH Aggregation + Direct Transmission to BS
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
    results.label   = 'EWMA-Detect';
end
