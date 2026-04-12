%% elect_ch_proposed.m — Jamming-Aware CH Election (Eq. 6)
% Computes CHScore for all alive nodes and greedily elects K CHs
% with spatial exclusion to ensure coverage across the field.
% Called every K_elec rounds inside the main loop.
%
% Inputs:
%   x, y         — node position vectors (m), length N
%   alive        — logical vector, length N
%   energy       — residual energy vector (J), length N
%   JR           — jamming risk vector, length N
%   dist_to_BS   — precomputed distance to BS vector (m), length N
%   E0           — initial energy (J)
%   d_max        — diagonal of deployment area (m)
%   r_c          — communication range for neighbor counting (m)
%   r_exc        — CH exclusion radius for spatial spread (m)
%   alpha, beta, gamma_, delta — CHScore weighting coefficients
%   r_tx         — hard transmission range limit (m); nodes farther than this
%                  from every CH are stranded (CH_assign stays 0)
%
% Outputs:
%   is_CH        — logical vector, 1=elected CH, length N
%   CH_assign    — cluster assignment vector, CH index for each node, length N
%                  0 means stranded (no CH within r_tx)

function [is_CH, CH_assign] = elect_ch_proposed(x, y, alive, energy, JR, ...
    dist_to_BS, E0, d_max, r_c, r_exc, alpha, beta, gamma_, delta, r_tx)

    N = length(x);
    is_CH     = false(1, N);
    CH_assign = zeros(1, N);

    %% Step 1 — Compute neighbor count |N_i(t)| for all alive nodes
    % Build pairwise distance matrix between all alive nodes
    alive_idx = find(alive);
    n_alive   = length(alive_idx);

    % Pairwise distances among alive nodes (vectorized)
    dx = x(alive_idx)' - x(alive_idx);   % n_alive x n_alive
    dy = y(alive_idx)' - y(alive_idx);
    D  = sqrt(dx.^2 + dy.^2);

    % Neighbor count: nodes within r_c, excluding self (diagonal is 0)
    neighbor_count          = zeros(1, N);
    counts                  = sum(D <= r_c, 2)' - 1;   % subtract self
    neighbor_count(alive_idx) = counts;

    N_max = max(neighbor_count(alive_idx));   % maximum neighbor count this round
    if N_max == 0; N_max = 1; end             % avoid division by zero

    %% Step 2 — Compute CHScore for all alive nodes (Eq. 6)
    score = zeros(1, N);
    for i = alive_idx
        score(i) = alpha * (energy(i) / E0) ...
                 + beta  * (neighbor_count(i) / N_max) ...
                 - gamma_* JR(i) ...
                 - delta * (dist_to_BS(i) / d_max);
    end

    %% Step 3 — Determine K: dynamic CH count (LEACH standard)
    p_CH = 0.05;
    K    = max(1, round(p_CH * n_alive));   % at least 1 CH always

    %% Step 4 — Greedy spatial election with exclusion radius r_exc
    % Sort alive nodes by score descending
    [~, sorted_idx] = sort(score(alive_idx), 'descend');
    sorted_nodes    = alive_idx(sorted_idx);

    eligible = true(1, N);   % tracks which nodes are still eligible
    n_elected = 0;

    for i = 1:length(sorted_nodes)
        if n_elected >= K; break; end

        node = sorted_nodes(i);
        if ~eligible(node); continue; end

        % Elect this node as CH
        is_CH(node) = true;
        n_elected   = n_elected + 1;

        % Suppress all nodes within r_exc of this CH
        dist_to_new_CH = sqrt((x - x(node)).^2 + (y - y(node)).^2);
        eligible(dist_to_new_CH <= r_exc) = false;
    end

    %% Step 5 — Cluster Assignment
    % Each alive non-CH node joins nearest CH within r_tx.
    % Nodes beyond r_tx from every CH are stranded: CH_assign stays 0.
    CH_idx = find(is_CH);
    for i = alive_idx
        if is_CH(i); continue; end
        dist_to_CHs       = sqrt((x(i) - x(CH_idx)).^2 + (y(i) - y(CH_idx)).^2);
        [min_d, nearest]  = min(dist_to_CHs);
        if min_d <= r_tx
            CH_assign(i) = CH_idx(nearest);
        end
        % else CH_assign(i) remains 0 — stranded node
    end

end