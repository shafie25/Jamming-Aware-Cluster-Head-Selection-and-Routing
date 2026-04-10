%% run_proposed.m — Proposed Jamming-Aware CH Selection and Routing
% Main round-by-round simulation loop for the proposed scheme.
% Calls all Layer 1 and Layer 2 modules each round.
% Returns per-round metrics for plotting.
%
% Inputs:  all variables from main.m workspace (loaded via config + init)
% Outputs: results_proposed struct with per-round metrics

function results = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
    E0, d_max, T, K_elec, M, lambda, r_c, r_exc, ...
    alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L)

    N = length(x);

    %% Initialize node state
    energy   = E0 * ones(1, N);
    PDR_ewma = ones(1, N);
    JR       = zeros(1, N);
    alive    = true(1, N);
    is_CH    = false(1, N);
    CH_assign = zeros(1, N);

    %% Initialize metrics storage
    PDR_per_round        = zeros(1, T);
    energy_per_round     = zeros(1, T);
    delay_per_round      = zeros(1, T);
    alive_per_round      = zeros(1, T);
    t_death              = NaN;   % round of first node death

    %% Round Loop
    for t = 1:T

        %% --- CH Election (every K_elec rounds) ---
        if mod(t, K_elec) == 0 || t == 1
            [is_CH, CH_assign] = elect_ch_proposed(x, y, alive, energy, JR, ...
                dist_to_BS, E0, d_max, r_c, r_exc, alpha, beta, gamma_, delta);

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
                d_to_CH = sqrt((x(i) - x(CH_assign(i)))^2 + (y(i) - y(CH_assign(i)))^2);
                energy(i) = energy(i) - compute_energy('overhead', L, E_elec, E_amp, E_da, d_to_CH, 0);
            end
        end

        %% --- Packet Success Probability ---
        p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);

        %% --- Member Transmission + JR Update ---
        [PDR_ewma, JR] = update_jamming_risk(p, alive, is_CH, PDR_ewma, JR, M, lambda);

        % Deduct transmission energy for member nodes
        for i = find(alive & ~is_CH)
            ch = CH_assign(i);
            if ch == 0; continue; end
            d_to_CH   = sqrt((x(i) - x(ch))^2 + (y(i) - y(ch))^2);
            energy(i) = energy(i) - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_CH, 0);
            energy(ch)= energy(ch) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
        end

        %% --- CH Aggregation + Routing ---
        [paths, hop_counts] = route_dijkstra(x, y, is_CH, JR, BS, ...
            phi1, phi2, phi3, E_amp, L);

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

            % Count packets received this round for PDR metric
            p_members  = p(members_c);
            recv_count = sum(rand(M, n_members) <= p_members, 'all');
            total_recv = total_recv + recv_count;
            total_sent = total_sent + n_members * M;

            % Route along optimal path, deduct energy per hop
            path = paths{c};
            if isempty(path); continue; end
            for h = 1:length(path) - 1
                ni = path(h);
                nj = path(h+1);
                if nj == N + 1
                    % Last hop to BS — BS has infinite energy
                    d_hop = sqrt((x(ni) - BS(1))^2 + (y(ni) - BS(2))^2);
                    energy(ni) = energy(ni) - compute_energy('tx', L, E_elec, E_amp, E_da, d_hop, 0);
                else
                    d_hop = sqrt((x(ni) - x(nj))^2 + (y(ni) - y(nj))^2);
                    energy(ni) = energy(ni) - compute_energy('tx', L, E_elec, E_amp, E_da, d_hop, 0);
                    energy(nj) = energy(nj) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
                end
            end

            total_delay = total_delay + hop_counts(c);
            n_CH_active = n_CH_active + 1;
        end

        %% --- Record Metrics ---
        PDR_per_round(t)    = (total_sent > 0) * total_recv / max(total_sent, 1);
        energy_per_round(t) = sum(energy(alive));
        delay_per_round(t)  = (n_CH_active > 0) * total_delay / max(n_CH_active, 1);
        alive_per_round(t)  = sum(alive);

        %% --- Node Death Check ---
        newly_dead = alive & (energy <= 0);
        if any(newly_dead) && isnan(t_death)
            t_death = t;   % record first node death round
        end
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