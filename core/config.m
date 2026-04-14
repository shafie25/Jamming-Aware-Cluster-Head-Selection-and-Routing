%% config.m — Simulation Parameters
% All parameters are defined here. Every other file calls config.m first.
% Units are noted in comments throughout.

%% Network Setup
% 100 nodes is the canonical WSN simulation size — large enough for
% meaningful clustering, small enough to run fast.
N        = 100;               % number of sensor nodes
area     = 100;               % field side length (m), giving 100x100m deployment
BS       = [area/2, area/2];  % base station at center — minimizes average routing
                              % distance and lets jamming avoidance be the 
                              % dominant differentiator in results
E0       = 0.5;               % initial energy per node (J) — 0.5J gives meaningful
                              % network lifetime before first node death (~500-1000 rounds)

%% Jammer Model
% r_j sized so jammer threatens 1-2 CHs per round, not zero and not all.
% With 5 CHs over 100x100m, 20m radius is the sweet spot.
r_j      = 20;                % jamming radius (m)
kappa    = 10;                 % jamming decay constant (unitless) — at boundary:
                              % no effect. At center: p drops to p_base*e^{-10} ~ 0.00004
p_base   = 0.95;              % baseline packet success probability outside jamming (unitless)

%% Round Control
T        = 1000;              % total simulation rounds
K_elec   = 10;                % CH re-election interval (rounds) — frequent enough
                              % to respond to UAV movement, EWMA has time to stabilize
                              % Note: K_elec used to avoid confusion with K (CH count)
M        = 10;                % burst size (packets/round) — gives PDR resolution of
                              % 0.1 steps {0, 0.1, ..., 1.0}, enough for smooth EWMA
p_CH     = 0.05;              % target fraction of nodes to elect as CHs (LEACH standard)
                              % K = round(p_CH * N_alive) computed dynamically each election
r_c      = 15;                % communication range for neighbor counting (m) — used in
                              % CHScore beta term to count nodes within range
r_exc    = 25;                % CH exclusion radius (m) — minimum spatial separation
                              % between elected CHs. Derived: area/K = 2000m^2 per CH,
                              % r = sqrt(2000/pi) ~ 25m. Guarantees spatial spread.
r_tx     = 50;                % hard transmission range limit (m) — member nodes beyond
                              % this distance from every CH are stranded (cannot join any
                              % cluster). Their M packets count in PDR denominator as lost.
                              % ~2x average CH-to-member distance in healthy network.

%% EWMA Smoothing
% lambda=0.6 — recent observations dominate but one anomalous round
% won't flip JR. High enough to track UAV movement, low enough to smooth noise.
lambda   = 0.6;               % EWMA smoothing constant (unitless), in (0,1)

%% CHScore Weights (must reflect paper claim: gamma_ weighted most heavily)
% These sum to 1.0 for interpretability. Tune after seeing initial results.
% alpha: energy,  beta: connectivity,  gamma_: jamming risk,  delta: BS distance
alpha    = 0.35;              % weight on residual energy term
beta     = 0.20;              % weight on neighbor connectivity term
gamma_   = 0.35;              % weight on jamming risk penalty (core contribution)
delta    = 0.10;              % weight on distance-to-BS penalty
% Sanity check: alpha + beta + gamma_ + delta = 1.0

%% Routing Weights
% phi2=1 lets physical constants carry the energy scaling naturally.
phi2 = 1;                     % energy term scale (unitless) — let epsilon_amp*L*d^2 dominate
phi1 = 1e-4;                  % fixed per-hop penalty (J) — ~18% of avg hop cost, nudges path length
phi3 = 5e-4;                  % JR penalty scale (J) — matches avg hop cost so JR meaningfully
                              % influences routing without dominating energy term

%% Radio Energy Model (Heinzelman et al. LEACH standard values)
% These are empirical constants from the original LEACH paper, not physical constants.
E_elec   = 50e-9;             % Tx/Rx circuit energy (J/bit)
E_amp    = 100e-12;           % amplifier energy (J/bit/m^2) — scales with d^2
E_da     = 5e-9;              % data aggregation energy at CH (J/bit)
L        = 4000;              % packet length (bits) — 500 bytes, standard WSN size