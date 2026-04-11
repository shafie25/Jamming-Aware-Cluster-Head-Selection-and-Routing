%% update_jamming_risk.m — Jamming Risk Estimation (Eq. 3, 4, 5)
% Computes instantaneous PDR from packet burst, updates EWMA-smoothed PDR,
% and derives jamming risk score for all alive member nodes.
% Called every round inside the main loop.
%
% Inputs:
%   p        — packet success probability vector, length N (from compute_packet_success)
%   alive    — logical vector, length N
%   is_CH    — logical vector, length N (CHs do not estimate JR as members)
%   PDR_ewma — current EWMA-smoothed PDR vector, length N
%   JR       — current jamming risk vector, length N
%   M        — burst size (packets/round)
%   lambda   — EWMA smoothing constant
%
% Outputs:
%   PDR_ewma — updated EWMA-smoothed PDR vector, length N
%   JR       — updated jamming risk vector, length N

function [PDR_ewma, JR] = update_jamming_risk(p, alive, is_CH, PDR_ewma, JR, M, lambda)

    % Only member nodes estimate JR — CHs are receiving, not transmitting bursts
    members = alive & ~is_CH;

    %% Eq. (3) — Instantaneous PDR from burst of M packets
    % Each packet succeeds independently with probability p_i(t)
    % Draw M uniform samples per node; count successes
    P_recv = sum(rand(M, sum(members)) <= p(members), 1);   % successes per member node
    PDR_inst = P_recv / M;                                   % instantaneous PDR in [0,1]

    %% Eq. (4) — EWMA-smoothed PDR
    PDR_ewma(members) = lambda * PDR_inst + (1 - lambda) * PDR_ewma(members);

    %% Eq. (5) — Jamming Risk Score
    JR(members) = 1 - PDR_ewma(members);

end