%clear all
%close all
%clc

N = 100;                                                                    % 100 sensor nodes
L = 100;                                                                    % length of the square field

%% Random deployment of sensor nodes with sink node in the center
x_loc = L.*rand(N, 1); y_loc = L.*rand(N, 1);                               % x and y location coordinates of sensor nodes
x_sink = L/2; y_sink = L/2; 

E = 0.5.*ones(N,1);                                                         % Initial energy for each node in Joules                                                
E_elec = 50*10^-9;                                                          % J/bit
E_da = 5*10^-9;                                                             % J/bit
S_data = 2000;                                                              % bits
epsilon_fs = 10*10^-12;                                                     % J/bit/m^2
epsilon_mp = 0.0013*10^-12;                                                 % J/bit/m^4
n_fs = 2;
n_mp = 4;
d_o = sqrt(epsilon_fs/epsilon_mp);

R = 3000;                                                                   % number of rounds
P = 0.05;                                                                    % desired percentage of CHs

G = ones(N,1);                                                              % any node can become a CH
alive = ones(N,1);                                                          % all nodes are alive

for ii = 1:R
    ii

    %% CH Selection
    random_number = rand(N, 1);
    T = P/(1-0.2*mod(ii,1/P));
    T = G.*T;
    CH_binary = random_number < T;
    CH_idx = find(CH_binary);                                               % CHs
    if length(CH_idx)==0                                                    % when the remaining # of sensor nodes is small, and no CHs are selected.. assume all will send data directly to sink >> as if all are CHs, receiving nothing, and aggregating nothing
        CH_idx = find(alive(:,ii));                                         
    end
    dead_idx = find(~alive(:,ii));                                          % dead nodes
    SN_idx = find((~CH_binary).*alive(:,ii));                               % basic sensor nodes: sensor nodes which are non-CHs and not dead

    %% Clustering
    D = sqrt((x_loc(CH_idx)-(x_loc(SN_idx))').^2 + (y_loc(CH_idx)-(y_loc(SN_idx))').^2);    % distance matrix CHxSN
    [D_sn_ch idx] = min(D);                                                                 % SN->CH distance; idx of CH for each SN (1,2,...K), where K is the # of CHs
    D_sn_ch = D_sn_ch';
    idx = idx';

    %% Sense and TX data -> energy spending 

    % basic sensor nodes
    n = n_fs.*(D_sn_ch<d_o) + n_mp.* (D_sn_ch>=d_o);                                        % choosing n
    epsilon = epsilon_fs.*(D_sn_ch<d_o) + epsilon_mp.* (D_sn_ch>=d_o);                      % choosing epsilon

    E(SN_idx,ii+1) = E(SN_idx,ii) - S_data*(E_elec + epsilon.*D_sn_ch.^n);    

    % CHs
    D_ch_sink = sqrt((x_loc(CH_idx)-x_sink).^2 + (y_loc(CH_idx)-y_sink).^2);    % distance matrix CH to sink
    N_c = [];                                                                   % number of cluster members in each cluster k
    for k = 1:length(CH_idx)
        N_c(k,1) = sum(idx == k) + 1;
    end

    n = n_fs.*(D_ch_sink<d_o) + n_mp.* (D_ch_sink>=d_o);                                        % choosing n
    epsilon = epsilon_fs.*(D_ch_sink<d_o) + epsilon_mp.* (D_ch_sink>=d_o);                      % choosing epsilon

    E(CH_idx,ii+1) = E(CH_idx,ii) - ((N_c-1).*S_data.*E_elec + E_da*S_data.*N_c + S_data*(E_elec + epsilon.*D_ch_sink.^n)); 

    % Dead nodes
    E(dead_idx,ii+1) = zeros(length(dead_idx),1);                                               % the rows of dead nodes are to be filled with zeros - not to keep them empty >> basically they dont have any energy - they are dead

    %% Check if nodes dies 
    alive(:,ii+1) = E(:,ii+1)>0; 

    %% update G for CHs
    if mod(ii,1/P) == 0                                                     % if the epoch is over, restart LEACH                        
        G = ones(N,1);
    else
        G(CH_idx) = zeros(length(CH_idx),1);                                % else, mark nodes who acted as CHs previously within this epoch with zeros
    end
    G = G.*alive(:,ii+1);                                                   % dead nodes never can be CHs anymore

end
E = E.*(E>=0);                                                              % make sure energy is never negative - node could have lost energy more than it has.... this means link failure

E_init_WSN = sum(E(:,1));                                                   % the overall initial WSN energy (i.e., energy of all nodes from beginning of first round)
Ep = sum(E)./E_init_WSN.*100;                                               % percentage of energy from the initial available one in the WSN

%figure;
%plot(0:R,Ep)                                                                % overall remaining energy in the WSN every round 
%grid on, box on, xlabel('Number of rounds'), ylabel('Total remaining percentage of the WSN energy (%)') 


