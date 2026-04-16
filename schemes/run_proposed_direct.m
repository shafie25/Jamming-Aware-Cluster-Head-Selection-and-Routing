%% run_proposed_direct.m — Proposed Scheme with Direct CH-to-BS (no Dijkstra)
% Identical to run_proposed.m except inter-cluster routing is replaced by
% direct CH→BS transmission for every CH. Used to isolate how much of the
% proposed scheme's performance comes from JR-aware CH election alone vs
% the combination of election + Dijkstra routing.
%
% Inputs/Outputs: identical signature to run_proposed.m

function results = run_proposed_direct(x, y, BS, J_x, J_y, dist_to_BS, ...
    E0, d_max, T, K_elec, M, lambda, p_CH, r_c, r_exc, ...
    alpha, beta, gamma_, delta, ~, ~, ~, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx)

    N = length(x);

    energy    = E0 * ones(1, N);
    PDR_ewma  = ones(1, N);
    JR        = zeros(1, N);
    alive     = true(1, N);
    is_CH     = false(1, N);
    CH_assign = zeros(1, N);
    ch_died_last_round = false;

    PDR_per_round    = zeros(1, T);
    energy_per_round = zeros(1, T);
    delay_per_round  = zeros(1, T);
    alive_per_round  = zeros(1, T);
    t_death          = NaN;

    for t = 1:T

        %% CH Election (same logic as run_proposed)
        need_regular_election   = (mod(t, K_elec) == 0 || t == 1);
        need_emergency_election = ~need_regular_election && (ch_died_last_round || ~any(is_CH & alive));

        if need_regular_election || need_emergency_election
            [is_CH, CH_assign] = elect_ch_proposed(x, y, alive, energy, JR, ...
                dist_to_BS, E0, d_max, p_CH, r_c, r_exc, alpha, beta, gamma_, delta, r_tx);

            CH_idx = find(is_CH);
            for c = CH_idx
                members_c = find(CH_assign == c & alive);
                if isempty(members_c); continue; end
                avg_d = mean(sqrt((x(members_c) - x(c)).^2 + (y(members_c) - y(c)).^2));
                energy(c) = energy(c) - compute_energy('overhead', L, E_elec, E_amp, E_da, avg_d, 0);
            end
            for i = find(alive & ~is_CH)
                if CH_assign(i) == 0; continue; end
                d_to_CH = sqrt((x(i) - x(CH_assign(i)))^2 + (y(i) - y(CH_assign(i)))^2);
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
            d_to_CH   = sqrt((x(i) - x(ch))^2 + (y(i) - y(ch))^2);
            energy(i) = energy(i) - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_CH, 0);
            energy(ch)= energy(ch) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
        end

        %% CH Aggregation + DIRECT transmission to BS (no Dijkstra)
        total_recv  = 0;
        total_sent  = 0;
        n_CH_active = 0;

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

            % Direct CH → BS (no relay, no Dijkstra)
            d_to_BS_c = sqrt((x(c) - BS(1))^2 + (y(c) - BS(2))^2);
            energy(c) = energy(c) - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_BS_c, 0);

            n_CH_active = n_CH_active + 1;
        end

        n_stranded = sum(alive & ~is_CH & (CH_assign == 0));
        total_sent = total_sent + n_stranded * M;

        PDR_per_round(t)    = (total_sent > 0) * total_recv / max(total_sent, 1);
        energy_per_round(t) = sum(energy(alive));
        delay_per_round(t)  = (n_CH_active > 0) * 1;   % always 1 hop
        alive_per_round(t)  = sum(alive);

        newly_dead = alive & (energy <= 0);
        if any(newly_dead) && isnan(t_death)
            t_death = t;
        end
        ch_died_last_round = any(is_CH & newly_dead);
        alive(newly_dead) = false;

        if ~any(alive); break; end
    end

    results.PDR     = PDR_per_round;
    results.energy  = energy_per_round;
    results.delay   = delay_per_round;
    results.alive   = alive_per_round;
    results.t_death = t_death;
    results.label   = 'Proposed (direct)';

end
