%% compute_packet_success.m — Packet Success Probability (Eq. 2)
% Computes p_i(t) for all nodes given current jammer position.
% Called every round inside the main loop.
%
% Inputs:
%   x, y     — node position vectors (m), length N
%   alive    — logical vector, 1=alive, length N
%   J_x_t   — jammer x-position at current round (m)
%   J_y_t   — jammer y-position at current round (m)
%   p_base  — baseline packet success probability (unitless)
%   kappa   — jamming decay constant (unitless)
%   r_j     — jamming radius (m)
%
% Output:
%   p        — packet success probability vector, length N
%              dead nodes get p=0

function p = compute_packet_success(x, y, alive, J_x_t, J_y_t, p_base, kappa, r_j)

    %% Distance from every node to jammer
    d_jammer = sqrt((x - J_x_t).^2 + (y - J_y_t).^2);   % (m), length N

    %% Case 1: nodes outside jamming radius → p = p_base
    p = p_base * ones(1, length(x));

    %% Case 2: nodes inside jamming radius → p = p_base * exp(-kappa*(1 - d/r_j))
    jammed = (d_jammer <= r_j) & alive;
    p(jammed) = p_base * exp(-kappa * (1 - d_jammer(jammed) / r_j));

    %% Dead nodes transmit nothing
    p(~alive) = 0;

end