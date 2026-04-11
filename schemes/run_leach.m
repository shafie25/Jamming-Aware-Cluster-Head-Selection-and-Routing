%% run_leach.m — Standard LEACH Baseline
% Implements the original LEACH protocol (Heinzelman et al.) as a baseline.
% Uses the same network topology, energy model, and jamming exposure as the
% proposed scheme so results are directly comparable.
%
% Key differences from proposed scheme:
%   - CH election is random/probabilistic (no jamming awareness)
%   - No inter-cluster routing — each CH transmits directly to BS
%   - No JR estimation or EWMA tracking
%
% Inputs:
%   x, y       — node positions (m), same vectors passed in from main.m
%   BS         — base station position [x, y] (m)
%   J_x, J_y   — jammer trajectory vectors, length T (same as proposed)
%   E0         — initial energy per node (J)
%   T          — total simulation rounds
%   M          — burst size (packets/round), for PDR computation
%   p_base     — baseline packet success probability
%   kappa      — jamming decay constant
%   r_j        — jamming radius (m)
%   E_elec     — circuit energy (J/bit)
%   E_amp      — amplifier energy (J/bit/m^2)
%   E_da       — data aggregation energy (J/bit)
%   L          — packet length (bits)
%
% Output:
%   results    — struct with fields: PDR, energy, delay, alive, t_death, label

function results = run_leach(x, y, BS, J_x, J_y, E0, T, M, ...
    p_base, kappa, r_j, E_elec, E_amp, E_da, L)

    N   = length(x);
    P   = 0.05;   % target CH fraction (LEACH standard)

    %% Initialize node state
    energy = E0 * ones(1, N);
    alive  = true(1, N);
    G      = true(1, N);   % eligibility: false for nodes that were CH this epoch

    %% Initialize metrics storage
    PDR_per_round    = zeros(1, T);
    energy_per_round = zeros(1, T);
    delay_per_round  = zeros(1, T);
    alive_per_round  = zeros(1, T);
    t_death          = NaN;

    %% Round Loop
    for t = 1:T

        %% --- CH Election (probabilistic, LEACH Eq.) ---
        T_thresh = zeros(1, N);
        denom = 1 - P * mod(t - 1, round(1/P));
        if denom <= 0; denom = 1e-6; end   % guard against divide-by-zero at epoch boundary
        T_thresh(G & alive) = P / denom;

        is_CH = (rand(1, N) < T_thresh) & alive;

        % If no CHs elected (can happen with small N_alive), make all alive nodes CHs
        if ~any(is_CH)
            is_CH = alive;
        end

        % Update epoch eligibility: nodes that served as CH this epoch are ineligible
        G(is_CH) = false;

        % Reset eligibility at epoch boundary
        if mod(t, round(1/P)) == 0
            G = alive;   % new epoch: all alive nodes eligible again
        end

        %% --- Cluster Assignment ---
        % Each alive non-CH joins the nearest CH
        CH_idx = find(is_CH);
        CH_assign = zeros(1, N);
        for i = find(alive & ~is_CH)
            d_to_CHs     = sqrt((x(i) - x(CH_idx)).^2 + (y(i) - y(CH_idx)).^2);
            [~, nearest] = min(d_to_CHs);
            CH_assign(i) = CH_idx(nearest);
        end

        %% --- Packet Success Probability (same jamming as proposed scheme) ---
        p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);

        %% --- Member Transmission Energy ---
        for i = find(alive & ~is_CH)
            ch = CH_assign(i);
            if ch == 0; continue; end
            d_to_CH    = sqrt((x(i) - x(ch))^2 + (y(i) - y(ch))^2);
            energy(i)  = energy(i)  - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_CH, 0);
            energy(ch) = energy(ch) - compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
        end

        %% --- CH Aggregation + Direct Transmission to BS ---
        total_recv  = 0;
        total_sent  = 0;
        n_CH_active = 0;

        for c = CH_idx
            members_c = find(CH_assign == c & alive);
            n_members = length(members_c);
            if n_members == 0; continue; end

            % Aggregation energy
            energy(c) = energy(c) - compute_energy('agg', L, E_elec, E_amp, E_da, 0, n_members);

            % PDR: stochastic packet draw for members
            p_members  = p(members_c);
            recv_count = sum(rand(M, n_members) <= p_members, 'all');
            total_recv = total_recv + recv_count;
            total_sent = total_sent + n_members * M;

            % Direct transmission to BS (no inter-cluster routing)
            d_to_BS   = sqrt((x(c) - BS(1))^2 + (y(c) - BS(2))^2);
            energy(c) = energy(c) - compute_energy('tx', L, E_elec, E_amp, E_da, d_to_BS, 0);

            n_CH_active = n_CH_active + 1;
        end

        %% --- Record Metrics ---
        PDR_per_round(t)    = (total_sent > 0) * total_recv / max(total_sent, 1);
        energy_per_round(t) = sum(energy(alive));
        delay_per_round(t)  = (n_CH_active > 0) * 1;   % LEACH is always 1 hop CH->BS
        alive_per_round(t)  = sum(alive);

        %% --- Node Death Check ---
        newly_dead = alive & (energy <= 0);
        if any(newly_dead) && isnan(t_death)
            t_death = t;
        end
        alive(newly_dead) = false;
        G(newly_dead)     = false;   % dead nodes can never be CHs

        if ~any(alive); break; end
    end

    %% Package results
    results.PDR     = PDR_per_round;
    results.energy  = energy_per_round;
    results.delay   = delay_per_round;
    results.alive   = alive_per_round;
    results.t_death = t_death;
    results.label   = 'Standard LEACH';

end
