%% init_network.m — Initialize Network State
% Called once at the start of main.m before any rounds run.
% Outputs all node state vectors of length N.

config;   % load all simulation parameters

%% Node Positions
% Uniformly random deployment over area x area field
x = rand(1, N) * area;   % x-coordinates (m)
y = rand(1, N) * area;   % y-coordinates (m)

%% Precompute Static Distances
% Nodes are static so distance to BS never changes — compute once here
dist_to_BS = sqrt((x - BS(1)).^2 + (y - BS(2)).^2);   % (m)

%% Initial Node State
energy   = E0 * ones(1, N);   % all nodes start at full energy (J)
PDR_ewma = ones(1, N);        % no jamming observed yet, PDR starts at 1
JR       = zeros(1, N);       % jamming risk starts at 0
alive    = true(1, N);        % all nodes alive at t=0

%% Precompute Max Possible Distance (for CHScore normalization)
% d_max is the diagonal of the deployment area
d_max = sqrt(2) * area;       % (m)