%% route_dijkstra.m — Jamming-Aware Inter-Cluster Routing (Eq. 7, 8)
% Builds weighted inter-cluster graph and finds minimum cost path
% from a source CH to the sink using Dijkstra's algorithm.
% Called every round for each CH in the main loop.
%
% Inputs:
%   x, y       — node position vectors (m), length N
%   is_CH      — logical vector, length N
%   JR         — jamming risk vector, length N
%   BS         — sink position [x, y] (m)
%   phi1       — fixed per-hop penalty (J)
%   phi2       — energy term weight (unitless)
%   phi3       — JR penalty weight (J)
%   E_amp      — amplifier energy constant (J/bit/m^2)
%   L          — packet length (bits)
%
% Output:
%   paths      — cell array of length N, paths{c} is the ordered list
%                of node indices from CH c to BS (including BS as last step)
%   hop_counts — vector of length N, number of hops for each CH to BS

function [paths, hop_counts] = route_dijkstra(x, y, is_CH, JR, BS, ...
    phi1, phi2, phi3, E_amp, L, r_tx)

    CH_idx = find(is_CH);
    N      = length(x);

    % Augment node set with BS as node N+1
    x_aug = [x, BS(1)];
    y_aug = [y, BS(2)];
    JR_aug = [JR, 0];        % BS has no jamming risk
    n_aug  = N + 1;
    BS_node = N + 1;

    % All nodes that can appear in inter-cluster graph: CHs + BS
    graph_nodes = [CH_idx, BS_node];
    n_graph     = length(graph_nodes);

    % Map from augmented index to graph index
    graph_map = zeros(1, n_aug);
    for g = 1:n_graph
        graph_map(graph_nodes(g)) = g;
    end

    %% Build cost matrix for inter-cluster graph
    % C(i,j) = phi1 + phi2*E_amp*L*d^2 + phi3*JR_j
    cost = inf(n_graph, n_graph);
    for gi = 1:n_graph
        for gj = 1:n_graph
            if gi == gj; continue; end
            ni = graph_nodes(gi);
            nj = graph_nodes(gj);
            d2 = (x_aug(ni) - x_aug(nj))^2 + (y_aug(ni) - y_aug(nj))^2;
            if sqrt(d2) > r_tx; continue; end   % no link beyond radio range
            cost(gi, gj) = phi1 + phi2 * E_amp * L * d2 + phi3 * JR_aug(nj);
        end
    end

    %% Run Dijkstra from each CH to BS
    paths      = cell(1, N);
    hop_counts = zeros(1, N);

    BS_g = graph_map(BS_node);   % BS index in graph

    for c = 1:length(CH_idx)
        src_node = CH_idx(c);
        src_g    = graph_map(src_node);

        % Dijkstra initialization
        dist_g = inf(1, n_graph);
        prev_g = zeros(1, n_graph);
        visited = false(1, n_graph);
        dist_g(src_g) = 0;

        for iter = 1:n_graph
            % Find unvisited node with minimum distance
            tmp = dist_g;
            tmp(visited) = inf;
            [~, u] = min(tmp);
            if isinf(dist_g(u)); break; end
            visited(u) = true;
            if u == BS_g; break; end

            % Relax neighbors
            for v = 1:n_graph
                if visited(v); continue; end
                new_dist = dist_g(u) + cost(u, v);
                if new_dist < dist_g(v)
                    dist_g(v) = new_dist;
                    prev_g(v) = u;
                end
            end
        end

        % Reconstruct path from BS back to source (append then reverse)
        path_g  = zeros(1, n_graph);
        path_len = 0;
        cur = BS_g;
        while cur ~= 0
            path_len = path_len + 1;
            path_g(path_len) = cur;
            cur = prev_g(cur);
        end
        path_g = fliplr(path_g(1:path_len));

        % Convert graph indices back to node indices
        path_nodes = graph_nodes(path_g);
        paths{src_node} = path_nodes;
        hop_counts(src_node) = length(path_nodes) - 1;   % hops = nodes - 1
    end

end