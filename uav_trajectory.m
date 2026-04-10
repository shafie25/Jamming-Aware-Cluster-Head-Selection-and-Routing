%% uav_trajectory.m — UAV Jammer Trajectory
% Precomputes the jammer position J(t) for all T rounds.
% Circular orbit centered at the field center, inside the deployment area.
% Called once at the start of main.m after init_network.m.

config;   % load all simulation parameters

%% Orbit Parameters
orbit_center = [area/2, area/2];   % orbit center coincides with field center (m)
orbit_radius = 35;                  % orbit radius (m) — keeps UAV inside 100x100m field
omega        = 2*pi / 50;          % angular speed (rad/round) — one full orbit per 50 rounds
                                    % every K_elec=10 rounds the jammer moves 72 degrees,
                                    % meaningfully threatening a different cluster region

%% Precompute Trajectory for All Rounds
% J_x(t) and J_y(t) are vectors of length T giving jammer position each round.
% Precomputing avoids recomputing trigonometry inside the round loop.
t_vec = 1:T;                                             % round indices
J_x   = orbit_center(1) + orbit_radius * cos(omega * t_vec);   % x-position (m)
J_y   = orbit_center(2) + orbit_radius * sin(omega * t_vec);   % y-position (m)

% J_x and J_y are used in the main round loop as J_x(t), J_y(t)