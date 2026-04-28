%% run_proposed.m — Proposed Jamming-Aware CH Selection and Routing
% Main round-by-round simulation loop for the proposed scheme.
% Calls all Layer 1 and Layer 2 modules each round.
% Returns per-round metrics for plotting.
%
% Inputs:  all variables from main.m workspace (loaded via config + init)
% Outputs: results_proposed struct with per-round metrics

function results = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
    E0, d_max, T, K_elec, M, lambda, p_CH, r_c, r_exc, ...
    alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx)

    N       = length(x);
    N_sleep = 5;   % rounds to sleep after going silent (covers jammer arc tail)

    %% Initialize node state
    energy    = E0 * ones(1, N);
    PDR_ewma  = ones(1, N);
    JR        = zeros(1, N);
    alive     = true(1, N);
    is_CH     = false(1, N);
    CH_assign = zeros(1, N);
    sleep_ctr = zeros(1, N);   % rounds remaining in sleep (0 = active)
    ch_died_last_round = false;

    %% Initialize metrics storage
    PDR_per_round        = zeros(1, T);
    energy_per_round     = zeros(1, T);
    delay_per_round      = zeros(1, T);
    alive_per_round      = zeros(1, T);
    t_death              = NaN;   % round of first node death

    %% Round Loop
    for t = 1:T

        %% --- CH Election (every K_elec rounds) ---
        need_regular_election = (mod(t, K_elec) == 0 || t == 1);
        need_emergency_election = ~need_regular_election && (ch_died_last_round || ~any(is_CH & alive));

        if need_regular_election || need_emergency_election
            [is_CH, CH_assign] = elect_ch_proposed(x, y, alive, energy, JR, ...
                dist_to_BS, E0, d_max, p_CH, r_c, r_exc, alpha, beta, gamma_, delta, r_tx);

            % Overhead energy: CH broadcasts ADV, members send join request
            CH_idx = find(is_CH);
            for c = CH_idx
                % CH broadcasts to all members — use average distance
                members_c = find(CH_assign == c & alive);
                if isempty(members_c); continue; end
                avg_d = mean(sqrt((x(members_c) - x(c)).^2 + (y(members_c) - y(c)).^2));
                energy(c) = energy(c) - compute_energy('overhead', L, E_elec, E_amp, E_da, avg_d, 0);
            end
            for i = find(alive & ~is_CH)
                if CH_assign(i) == 0; continue; end   % stranded — no CH to join
                d_to_CH = sqrt((x(i) - x(CH_assign(i)))^2 + (y(i) - y(CH_assign(i)))^2);
                energy(i) = energy(i) - compute_energy('overhead', L, E_elec, E_amp, E_da, d_to_CH, 0);
            end
        end

        %% --- Packet Success Probability ---
        p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);

        %% --- JR Update (freeze sleeping nodes — no feedback while silent) ---
        PDR_ewma_frozen = PDR_ewma;
        JR_frozen       = JR;
        [PDR_ewma, JR] = update_jamming_risk(p, alive, is_CH, PDR_ewma, JR, M, lambda);

        sleeping = alive & (sleep_ctr > 0);
        PDR_ewma(sleeping) = PDR_ewma_frozen(sleeping);
        JR(sleeping)       = JR_frozen(sleeping);

        % Decrement counters; reset JR optimistically on wakeup
        sleep_ctr(sleeping) = sleep_ctr(sleeping) - 1;
        just_woke = sleeping & (sleep_ctr == 0);
        JR(just_woke)       = 0;
        PDR_ewma(just_woke) = 1;

        % M_eff = 0 when JR high enough — silent nodes excluded from PDR/energy
        M_eff = round(M * (1 - JR));

        % Nodes that just went silent start their sleep countdown
        newly_silent = alive & ~is_CH & (M_eff == 0) & (sleep_ctr == 0);
        sleep_ctr(newly_silent) = N_sleep;

        % Deduct transmission energy for member nodes, scaled by M_eff/M
        for i = find(alive & ~is_CH)
            ch = CH_assign(i);
            if ch == 0; continue; end
            scale     = M_eff(i) / M;
            d_to_CH   = sqrt((x(i) - x(ch))^2 + (y(i) - y(ch))^2);
            energy(i) = energy(i) - scale * compute_energy('tx', L, E_elec, E_amp, E_da, d_to_CH, 0);
            energy(ch)= energy(ch) - scale * compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
        end

        %% --- Stranded Node Relay ---
        % Isolated nodes (no CH within r_tx) attempt a 2-hop path:
        %   stranded → highest-energy non-CH member within r_tx → that member's CH
        % The relay packets then travel the CH's existing Dijkstra path to the BS.
        %
        % Three efficiency constraints:
        %   (1) JR filter: skip heavily jammed stranded nodes (JR >= 0.5) —
        %       two lossy hops compound badly and waste energy for near-zero gain.
        %   (2) Relay cap: each relay node serves at most one stranded node per round
        %       to prevent any single node from absorbing disproportionate load.
        %   (3) Amortised relay→CH energy: relay is already transmitting to its CH;
        %       only the new stranded→relay hop (TX + RX) is charged separately.
        relay_bonus = zeros(1, N);

        % Relay only activates after first node death. In healthy rounds, stranded
        % nodes are transient (re-election will cover them) and relay overhead
        % hurts lifetime more than it helps PDR. Post-FND, CH coverage degrades
        % permanently and relay recovers packets that would otherwise be lost.
        if ~isnan(t_death)
        relay_used  = false(1, N);

        for i = find(alive & ~is_CH & (CH_assign == 0) & (M_eff > 0) & (JR < 0.5))
            % Candidates: alive non-CH members within r_tx not yet used as relay
            d_i        = sqrt((x(i) - x).^2 + (y(i) - y).^2);
            candidates = find(alive & ~is_CH & (CH_assign > 0) & (d_i <= r_tx) & ~relay_used);
            if isempty(candidates); continue; end

            % Pick relay with highest residual energy
            [~, best] = max(energy(candidates));
            relay     = candidates(best);
            ch        = CH_assign(relay);

            % Charge only the stranded→relay hop; relay→CH is amortised
            s         = M_eff(i) / M;
            d_ir      = sqrt((x(i) - x(relay))^2 + (y(i) - y(relay))^2);
            energy(i)     = energy(i)     - s * compute_energy('tx', L, E_elec, E_amp, E_da, d_ir, 0);
            energy(relay) = energy(relay) - s * compute_energy('rx', L, E_elec, E_amp, E_da, 0,   0);

            % Two-hop channel loss
            hop1 = sum(rand(M_eff(i), 1) <= p(i));
            hop2 = sum(rand(hop1, 1) <= p(relay));
            relay_bonus(ch) = relay_bonus(ch) + hop2;
            relay_used(relay) = true;
        end
        end   % if ~isnan(t_death)

        %% --- CH Aggregation + Routing ---
        [paths, hop_counts] = route_dijkstra(x, y, is_CH, JR, BS, ...
            phi1, phi2, phi3, E_amp, L, r_tx);

        total_delay = 0;
        n_CH_active = 0;
        total_recv  = 0;
        total_sent  = 0;

        CH_idx = find(is_CH & alive);
        for c = CH_idx
            members_c = find(CH_assign == c & alive);
            n_members = length(members_c);
            if n_members == 0; continue; end

            % Aggregation energy
            energy(c) = energy(c) - compute_energy('agg', L, E_elec, E_amp, E_da, 0, n_members);

            % Count packets received — each member sends M_eff(i) packets
            p_members   = p(members_c);
            m_eff_c     = M_eff(members_c);
            recv_count  = 0;
            for mi = 1:n_members
                recv_count = recv_count + sum(rand(m_eff_c(mi), 1) <= p_members(mi));
            end
            total_sent = total_sent + sum(m_eff_c);

            % Route along optimal path: deduct energy per hop and apply channel loss
            path = paths{c};
            if isempty(path); continue; end
            surviving = recv_count + relay_bonus(c);
            for h = 1:length(path) - 1
                ni = path(h);
                nj = path(h+1);
                if nj == N + 1
                    % Last hop to BS — BS has infinite energy
                    d_hop = sqrt((x(ni) - BS(1))^2 + (y(ni) - BS(2))^2);
                    energy(ni) = energy(ni) - compute_energy('tx', L, E_elec, E_amp, E_da, d_hop, 0);
                    if surviving > 0
                        surviving = sum(rand(surviving, 1) <= p(ni));
                    end
                else
                    d_hop = sqrt((x(ni) - x(nj))^2 + (y(ni) - y(nj))^2);
                    energy(ni) = energy(ni) - compute_energy('tx', L, E_elec, E_amp, E_da, d_hop, 0);
                    energy(nj) = energy(nj) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
                    if surviving > 0
                        surviving = sum(rand(surviving, 1) <= p(ni));
                    end
                end
            end
            total_recv = total_recv + surviving;

            total_delay = total_delay + hop_counts(c);
            n_CH_active = n_CH_active + 1;
        end

        %% --- Stranded nodes: count their (reduced) packets as lost in denominator ---
        total_sent = total_sent + sum(M_eff(alive & ~is_CH & (CH_assign == 0)));

        %% --- Record Metrics ---
        PDR_per_round(t)    = (total_sent > 0) * total_recv / max(total_sent, 1);
        energy_per_round(t) = sum(energy(alive));
        delay_per_round(t)  = (n_CH_active > 0) * total_delay / max(n_CH_active, 1);
        alive_per_round(t)  = sum(alive);

        %% --- Node Death Check ---
        newly_dead = alive & (energy <= 0);
        if any(newly_dead) && isnan(t_death)
            t_death = t;
        end
        ch_died_last_round = any(is_CH & newly_dead);
        alive(newly_dead) = false;

        % If no alive nodes remain, stop early
        if ~any(alive); break; end
    end

    %% Package results
    results.PDR        = PDR_per_round;
    results.energy     = energy_per_round;
    results.delay      = delay_per_round;
    results.alive      = alive_per_round;
    results.t_death    = t_death;
    results.label      = 'Proposed';

end
