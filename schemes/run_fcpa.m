%% run_fcpa.m — FCPA Baseline (IPN-Aware Clustering + Cooperative Relay)
% Simplified adaptation of the FCPA strategy from:
%   López-Vilos et al., "Clustering-Based Energy-Efficient Self-Healing
%   Strategy for WSNs under Jamming Attacks," Sensors 2023.
%
% Key mechanisms retained from paper:
%   - IPN-gated CH election: nodes geometrically close to the jammer
%     (d < r_j) are ineligible as CH this round (Section III-B)
%   - Cooperative relay for jammed members: jammed member i finds the
%     cheapest non-jammed relay j reachable from both i and the CH, then
%     uses i → j → CH path (Section III-C, Eq. 7)
%   - Load-balanced relay weight: w_ij = (1/e_coop_ij) / sum_k(1/e_coop_ik)
%     relay selection = argmax(w) = argmin(e_coop) (Eq. 7)
%
% Run 024 change: CH election fires every K_elec rounds (plus emergency),
% matching proposed's cadence. The paper re-clusters only when the jammer
% moves or changes power; with a continuously orbiting UAV, firing every
% round was an unfair per-round overhead penalty.
%
% Energy/PDR accounting:
%   - PDR = member-to-CH packets delivered (consistent with run_proposed.m)
%   - Fixed M packet trials per alive node in total_sent denominator
%   - Relay nodes accumulate TX+RX forwarding costs via deferred delta
%
% Inputs: same workspace variables as run_proposed.m
% Output: results struct with PDR, energy, delay, alive, t_death, label

function results = run_fcpa(x, y, BS, J_x, J_y, E0, T, M, K_elec, ...
    p_CH, p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx)

    N = length(x);

    %% Initialize node state
    energy    = E0 * ones(1, N);
    alive     = true(1, N);
    G         = true(1, N);   % LEACH-style epoch eligibility
    is_CH     = false(1, N);
    CH_assign = zeros(1, N);
    ch_died_last_round = false;

    %% Initialize metrics storage
    PDR_per_round    = zeros(1, T);
    energy_per_round = zeros(1, T);
    delay_per_round  = zeros(1, T);
    alive_per_round  = zeros(1, T);
    t_death          = NaN;

    %% Precompute distances
    dist_to_BS = sqrt((x - BS(1)).^2 + (y - BS(2)).^2);
    dist_nodes = zeros(N, N);
    for i = 1:N
        for j = i+1:N
            d = sqrt((x(i)-x(j))^2 + (y(i)-y(j))^2);
            dist_nodes(i,j) = d;
            dist_nodes(j,i) = d;
        end
    end

    %% Round Loop
    for t = 1:T

        %% --- Channel quality and IPN gate (every round) ---
        p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);
        d_jammer   = sqrt((x - J_x(t)).^2 + (y - J_y(t)).^2);
        ipn_jammed = d_jammer < r_j;

        %% --- CH Election (every K_elec rounds + emergency) ---
        need_election = (mod(t, K_elec) == 0) || (t == 1) || ...
                        ch_died_last_round || ~any(is_CH & alive);

        energy_delta = zeros(1, N);

        if need_election
            denom = 1 - p_CH * mod(t-1, round(1/p_CH));
            if denom <= 0; denom = 1e-6; end
            T_thresh = zeros(1, N);
            T_thresh(G & alive & ~ipn_jammed) = p_CH / denom;

            is_CH = (rand(1, N) < T_thresh) & alive;

            % Fallback: if all eligible nodes are jammed or threshold yields no CH
            if ~any(is_CH)
                eligible = alive & ~ipn_jammed;
                if any(eligible)
                    is_CH = eligible & (rand(1, N) < p_CH);
                    if ~any(is_CH)
                        is_CH(find(eligible, 1)) = true;
                    end
                else
                    is_CH = alive;
                end
            end

            G(is_CH) = false;
            if mod(t, round(1/p_CH)) == 0
                G = alive;
            end

            %% --- Cluster Formation ---
            CH_idx    = find(is_CH);
            CH_assign = zeros(1, N);
            for i = find(alive & ~is_CH)
                d_to_CHs         = dist_nodes(i, CH_idx);
                [min_d, nearest] = min(d_to_CHs);
                if min_d <= r_tx
                    CH_assign(i) = CH_idx(nearest);
                end
            end

            %% --- CH election overhead (ADV + JOIN, same model as proposed) ---
            for c = CH_idx
                members_c = find(CH_assign == c & alive);
                if isempty(members_c); continue; end
                avg_d = mean(dist_nodes(c, members_c));
                energy_delta(c) = energy_delta(c) - compute_energy('overhead', L, E_elec, E_amp, E_da, avg_d, 0);
            end
            for i = find(alive & ~is_CH)
                if CH_assign(i) == 0; continue; end
                energy_delta(i) = energy_delta(i) - compute_energy('overhead', L, E_elec, E_amp, E_da, dist_nodes(i, CH_assign(i)), 0);
            end
        end

        %% --- Per-round data transmission ---
        total_recv = 0;
        total_sent = 0;
        total_hops = 0;
        n_routed   = 0;

        % Pool of potential relays: alive, non-CH, non-jammed nodes.
        relay_pool = find(alive & ~is_CH & ~ipn_jammed);
        CH_idx     = find(is_CH);

        %% --- Per-CH processing ---
        for c = CH_idx
            members_all    = find(alive & ~is_CH & (CH_assign == c));
            if isempty(members_all); continue; end

            direct_members = members_all(~ipn_jammed(members_all));
            jammed_members = members_all( ipn_jammed(members_all));
            ch_recv        = 0;

            %% Direct member → CH transmissions
            for i = direct_members
                total_sent = total_sent + M;
                d_ic       = dist_nodes(i, c);

                energy_delta(i) = energy_delta(i) - compute_energy('tx', L, E_elec, E_amp, E_da, d_ic, 0);
                energy_delta(c) = energy_delta(c) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);

                recv       = sum(rand(M, 1) <= p(i));
                ch_recv    = ch_recv + recv;
                total_hops = total_hops + 1;
                n_routed   = n_routed + 1;
            end

            %% Jammed member → relay → CH (cooperative relay, FCPA Eq. 7)
            for i = jammed_members
                total_sent = total_sent + M;

                % Candidates: relay_pool nodes reachable from i AND from CH c
                d_i_to_pool = dist_nodes(i, relay_pool);
                d_pool_to_c = dist_nodes(relay_pool, c)';
                candidates  = relay_pool( ...
                    relay_pool ~= i & ...
                    d_i_to_pool   <= r_tx & ...
                    d_pool_to_c   <= r_tx);

                if isempty(candidates)
                    continue;
                end

                % e_coop(j) = TX(i→j) + RX(j) + TX(j→c)
                e_coop = zeros(1, length(candidates));
                for k = 1:length(candidates)
                    j_node    = candidates(k);
                    d_ij      = dist_nodes(i, j_node);
                    d_jc      = dist_nodes(j_node, c);
                    e_coop(k) = compute_energy('tx', L, E_elec, E_amp, E_da, d_ij, 0) + ...
                                compute_energy('rx', L, E_elec, E_amp, E_da, 0,   0) + ...
                                compute_energy('tx', L, E_elec, E_amp, E_da, d_jc, 0);
                end

                % Select relay with lowest cooperative cost (argmin = argmax weight)
                [~, best_k] = min(e_coop);
                j_node = candidates(best_k);
                d_ij   = dist_nodes(i, j_node);
                d_jc   = dist_nodes(j_node, c);

                energy_delta(i)      = energy_delta(i)      - compute_energy('tx', L, E_elec, E_amp, E_da, d_ij, 0);
                energy_delta(j_node) = energy_delta(j_node) - compute_energy('rx', L, E_elec, E_amp, E_da, 0,   0);
                energy_delta(j_node) = energy_delta(j_node) - compute_energy('tx', L, E_elec, E_amp, E_da, d_jc, 0);
                energy_delta(c)      = energy_delta(c)      - compute_energy('rx', L, E_elec, E_amp, E_da, 0,   0);

                recv_at_relay = sum(rand(M, 1) <= p(i));
                if recv_at_relay > 0
                    recv_at_ch = sum(rand(recv_at_relay, 1) <= p(j_node));
                else
                    recv_at_ch = 0;
                end

                ch_recv    = ch_recv + recv_at_ch;
                total_hops = total_hops + 2;
                n_routed   = n_routed + 1;
            end

            %% CH→BS: gate PDR on radio range, charge TX energy regardless
            if ch_recv > 0 && dist_to_BS(c) <= r_tx
                total_recv = total_recv + sum(rand(ch_recv, 1) <= p(c));
            end

            n_members_c     = length(members_all);
            energy_delta(c) = energy_delta(c) - compute_energy('agg', L, E_elec, E_amp, E_da, 0, n_members_c);
            energy_delta(c) = energy_delta(c) - compute_energy('tx',  L, E_elec, E_amp, E_da, dist_to_BS(c), 0);
        end

        %% --- Stranded nodes: count M packet trials as lost ---
        n_stranded = sum(alive & ~is_CH & (CH_assign == 0));
        total_sent = total_sent + n_stranded * M;

        %% --- Apply energy deltas ---
        energy = energy + energy_delta;

        %% --- Record Metrics ---
        PDR_per_round(t)    = (total_sent > 0) * total_recv / max(total_sent, 1);
        energy_per_round(t) = sum(energy(alive));
        delay_per_round(t)  = (n_routed > 0) * total_hops / max(n_routed, 1);
        alive_per_round(t)  = sum(alive);

        %% --- Node Death Check ---
        newly_dead = alive & (energy <= 0);
        if any(newly_dead) && isnan(t_death)
            t_death = t;
        end
        ch_died_last_round = any(is_CH & newly_dead);
        alive(newly_dead)  = false;
        G(newly_dead)      = false;

        if ~any(alive); break; end
    end

    %% Package results
    results.PDR     = PDR_per_round;
    results.energy  = energy_per_round;
    results.delay   = delay_per_round;
    results.alive   = alive_per_round;
    results.t_death = t_death;
    results.label   = 'FCPA';

end
