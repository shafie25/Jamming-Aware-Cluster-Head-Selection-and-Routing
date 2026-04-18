%% run_tbc.m — TBC Baseline (Threshold-Based Countermeasure)
% Flat-topology WSN baseline adapted from:
%   Babitha B.S. et al., "Threshold-Based Jamming Attack Detection and
%   Cure Mechanism for Wireless Sensor Networks," IEEE MRIE 2025.
%
% Topology: flat multi-hop — no clustering, no CH election.
% Every alive node routes M packets to BS via shortest-distance path
% through alive, non-jammed neighbors within r_tx.
%
% Jamming detection (Eq. 3 adapted): instantaneous PDR < T_threshold →
%   node is flagged as jammed.
% Path reconfiguration (Eq. 6: P_new = P - P_malicious): jammed nodes are
%   excluded from the routing graph — packets cannot route through them,
%   and they do not originate transmissions that round.
% TKM (Section III-D) omitted: replay-attack specific, not applicable to
%   continuous UAV jamming.
%
% Energy model: TX at each forwarding hop + RX at each receiving hop.
%   Relay nodes accumulate forwarding costs from all traffic passing through.
%   Nodes with no path to BS (isolated or all neighbors jammed) lose packets.
%
% Inputs: same workspace variables as run_proposed.m / run_leach.m
% Output: results struct with PDR, energy, delay, alive, t_death, label

function results = run_tbc(x, y, BS, J_x, J_y, ...
    E0, T, M, p_base, kappa, r_j, E_elec, E_amp, L, r_tx)

    N           = length(x);
    T_threshold = 0.5;   % PDR < 50% → node flagged as jammed (Eq. 3 adapted)
    BS_idx      = N + 1; % virtual index for base station in routing graph

    %% Initialize node state
    energy = E0 * ones(1, N);
    alive  = true(1, N);

    %% Initialize metrics storage
    PDR_per_round    = zeros(1, T);
    energy_per_round = zeros(1, T);
    delay_per_round  = zeros(1, T);
    alive_per_round  = zeros(1, T);
    t_death          = NaN;

    %% Precompute inter-node distances and node-to-BS distances
    dist_nodes = zeros(N, N);
    for i = 1:N
        for j = i+1:N
            d = sqrt((x(i)-x(j))^2 + (y(i)-y(j))^2);
            dist_nodes(i,j) = d;
            dist_nodes(j,i) = d;
        end
    end
    dist_to_BS = sqrt((x - BS(1)).^2 + (y - BS(2)).^2);

    %% Round Loop
    for t = 1:T

        %% --- Packet Success Probability ---
        p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);

        %% --- Instantaneous PDR per node (Eq. 2: T_i = received/sent) ---
        T_i = zeros(1, N);
        for i = find(alive)
            T_i(i) = sum(rand(M, 1) <= p(i)) / M;
        end

        %% --- Jamming Detection (Eq. 3 adapted) ---
        jammed = alive & (T_i < T_threshold);

        %% --- Build Routing Graph (Eq. 6: exclude jammed nodes) ---
        % Nodes that can participate in routing: alive and not jammed.
        % Edges exist between nodes within r_tx of each other.
        % BS is always reachable as a sink (it is never jammed).
        routable = alive & ~jammed;

        % Dijkstra from BS outward to find shortest-distance path to BS
        % for every routable source node.
        % Graph nodes: 1..N (sensor nodes) + BS_idx (N+1).
        inf_dist = 1e9;
        D        = inf_dist * ones(1, N+1);
        visited  = false(1, N+1);
        next_hop = zeros(1, N+1);   % next hop toward BS for each node
        D(BS_idx) = 0;

        for iter = 1:N+1
            % Pick unvisited node with smallest tentative distance
            D_copy = D; D_copy(visited) = inf_dist;
            [~, u] = min(D_copy);
            if D(u) == inf_dist; break; end
            visited(u) = true;

            if u == BS_idx
                % Neighbors of BS: sensor nodes within r_tx of BS.
                % Edge weight penalises v by its own energy depletion — a
                % near-BS node low on energy gets a higher effective distance,
                % so Dijkstra prefers fresher alternatives.
                for v = find(routable)
                    if dist_to_BS(v) <= r_tx
                        e_penalty = E0 / max(energy(v), 1e-6);
                        new_d = dist_to_BS(v) * e_penalty;
                        if new_d < D(v)
                            D(v) = new_d;
                            next_hop(v) = BS_idx;
                        end
                    end
                end
            else
                % Neighbors of sensor node u: other routable nodes within r_tx.
                % Penalty is on u (the relay) — if u is low energy, v should
                % find a fresher relay instead. This naturally load-balances
                % relay burden across the field over time.
                e_penalty_u = E0 / max(energy(u), 1e-6);
                for v = find(routable)
                    if v == u; continue; end
                    if dist_nodes(u, v) <= r_tx
                        new_d = D(u) + dist_nodes(u, v) * e_penalty_u;
                        if new_d < D(v)
                            D(v) = new_d;
                            next_hop(v) = u;
                        end
                    end
                end
                % Can u reach BS directly?
                if dist_to_BS(u) <= r_tx
                    new_d = D(u) + dist_to_BS(u) * e_penalty_u;
                    if new_d < D(BS_idx)
                        D(BS_idx) = new_d;
                        next_hop(BS_idx) = u;
                    end
                end
            end
        end

        %% --- Packet Forwarding ---
        % Each alive, non-jammed node with a path to BS originates M packets.
        % Packets traverse hops; each hop costs TX at sender + RX at receiver.
        % Relay nodes accumulate forwarding energy from all traffic routed through.
        % Jammed nodes: M packets count as lost (no TX, no energy cost).

        % Count how many packets each node needs to forward (originating + relay).
        % We process per-source to correctly attribute energy to relay nodes.

        total_recv = 0;
        total_sent = 0;
        total_hops = 0;
        n_routed   = 0;

        % Temporary energy delta — apply all at once after the routing pass
        % so relay ordering doesn't affect who dies mid-round.
        energy_delta = zeros(1, N);

        for i = find(alive)
            total_sent = total_sent + M;   % all alive nodes had M packets to send

            if jammed(i)
                % Jammed: suppress transmission, packets lost (Eq. 6)
                continue;
            end

            if D(i) == inf_dist
                % No path to BS (isolated): packets lost
                continue;
            end

            % Trace the path from i to BS, accumulate energy costs
            % and count received packets (apply channel loss at each hop).
            recv_packets = M;   % start with M packets, lose some at each hop
            cur = i;
            hops = 0;

            while cur ~= BS_idx
                nh = next_hop(cur);
                if nh == 0; recv_packets = 0; break; end  % broken path guard

                % Distance of this hop
                if nh == BS_idx
                    d_hop = dist_to_BS(cur);
                else
                    d_hop = dist_nodes(cur, nh);
                end

                % TX energy at current node.
                % Scale by recv_packets/M so that a full burst (recv_packets==M)
                % costs exactly L*E_elec + L*E_amp*d^2 — the same convention used
                % by compute_energy('tx') in run_proposed/run_leach, where L is the
                % total round payload (M packets combined), not per-packet size.
                energy_delta(cur) = energy_delta(cur) - (recv_packets/M) * (L*E_elec + L*E_amp*d_hop^2);

                % RX energy at next hop (BS has infinite energy)
                if nh ~= BS_idx
                    energy_delta(nh) = energy_delta(nh) - (recv_packets/M) * L * E_elec;
                end

                % Channel loss at this hop — vectorised Bernoulli over surviving packets
                recv_packets = sum(rand(recv_packets, 1) <= p(cur));

                cur  = nh;
                hops = hops + 1;
                if hops > N; break; end  % loop guard
            end

            if cur == BS_idx
                total_recv = total_recv + recv_packets;
                total_hops = total_hops + hops;
                n_routed   = n_routed + 1;
            end
        end

        % Apply energy deltas
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
        alive(newly_dead) = false;

        if ~any(alive); break; end
    end

    %% Package results
    results.PDR     = PDR_per_round;
    results.energy  = energy_per_round;
    results.delay   = delay_per_round;
    results.alive   = alive_per_round;
    results.t_death = t_death;
    results.label   = 'TBC';

end
