%% compute_energy.m — LEACH Radio Energy Model
% Computes energy cost for a given radio operation.
% Called inside the round loop for member nodes, CHs, and overhead.
%
% Inputs:
%   type      — operation type: 'tx', 'rx', 'agg', 'overhead'
%   L         — packet length (bits)
%   E_elec    — circuit energy (J/bit)
%   E_amp     — amplifier energy (J/bit/m^2)
%   E_da      — data aggregation energy (J/bit)
%   d         — transmission distance (m), required for 'tx' only
%   n_members — number of cluster members, required for 'agg' only
%
% Output:
%   E         — energy cost of the operation (J)

function E = compute_energy(type, L, E_elec, E_amp, E_da, d, n_members)

    switch type

        case 'tx'
            % Member node transmitting L bits to CH over distance d
            % Or CH forwarding aggregated data to next hop over distance d
            % Energy grows quadratically with distance (free-space path loss)
            E = L * E_elec + L * E_amp * d^2;

        case 'rx'
            % Receiving L bits — no amplifier cost, circuit energy only
            E = L * E_elec;

        case 'agg'
            % CH aggregating data from n_members — pays E_da per bit per member
            % Total bits received = n_members * L
            E = n_members * L * E_da;

        case 'overhead'
            % Control messages: CH advertisement broadcasts, member join requests
            % Modeled as fixed small packet of 200 bits over distance d
            L_ctrl = 200;   % control packet size (bits)
            E = L_ctrl * E_elec + L_ctrl * E_amp * d^2;

        otherwise
            error('compute_energy: unknown type "%s". Use tx, rx, agg, overhead.', type);

    end

end