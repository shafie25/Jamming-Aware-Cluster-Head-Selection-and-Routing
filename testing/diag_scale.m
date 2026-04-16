%% diag_scale.m — Zero-PDR Cause Breakdown for Scaled Deployment (200x200m)
% Runs one seed of the proposed scheme at 200x200m parameters and classifies
% every zero-PDR round by cause. Used to diagnose whether zeros are from
% CHs=0, ALL_STRANDED, ALL_JAMMED, or stochastic bad luck.
%
% Run from project root: >> run('testing/diag_scale.m')

clc; clear;
addpath(genpath('.'));

%% Scaled parameters (must match run_scale_test.m)
rng(1);
config;
N            = 200;
area         = 200;
BS           = [100, 100];
orbit_radius = 70;
omega        = 2*pi / 50;
r_exc        = 35;
r_tx         = 75;
d_max        = sqrt(2) * area;

x = rand(1, N) * area;
y = rand(1, N) * area;
dist_to_BS = sqrt((x - BS(1)).^2 + (y - BS(2)).^2);
t_vec = 1:T;
J_x = BS(1) + orbit_radius * cos(omega * t_vec);
J_y = BS(2) + orbit_radius * sin(omega * t_vec);

energy    = E0 * ones(1, N);
PDR_ewma  = ones(1, N);
JR        = zeros(1, N);
alive     = true(1, N);
is_CH     = false(1, N);
CH_assign = zeros(1, N);
ch_died_last_round = false;
M_min = max(1, round(0.2 * M));
PDR_per_round = zeros(1, T);

cause_counts = struct('ALL_STRANDED', 0, 'CHs_ZERO', 0, 'ALL_JAMMED', 0, 'STOCHASTIC', 0);

fprintf('Zero-PDR cause breakdown — Seed 1, 200x200m, r_tx=%dm\n', r_tx);
fprintf('%-6s | %-5s | %-4s | %-8s | %-7s | %s\n', 'Round','Alive','CHs','Stranded','Members','Cause');
fprintf('%s\n', repmat('-', 1, 60));

for t = 1:T
    need_reg = (mod(t, K_elec) == 0 || t == 1);
    need_emg = ~need_reg && (ch_died_last_round || ~any(is_CH & alive));

    if need_reg || need_emg
        [is_CH, CH_assign] = elect_ch_proposed(x, y, alive, energy, JR, ...
            dist_to_BS, E0, d_max, p_CH, r_c, r_exc, alpha, beta, gamma_, delta, r_tx);
        for c = find(is_CH)
            mc = find(CH_assign == c & alive);
            if isempty(mc); continue; end
            ad = mean(sqrt((x(mc) - x(c)).^2 + (y(mc) - y(c)).^2));
            energy(c) = energy(c) - compute_energy('overhead', L, E_elec, E_amp, E_da, ad, 0);
        end
        for i = find(alive & ~is_CH)
            ch = CH_assign(i);
            if ch == 0; continue; end
            d_ch = sqrt((x(i) - x(ch))^2 + (y(i) - y(ch))^2);
            energy(i) = energy(i) - compute_energy('overhead', L, E_elec, E_amp, E_da, d_ch, 0);
        end
    end

    p = compute_packet_success(x, y, alive, J_x(t), J_y(t), p_base, kappa, r_j);
    [PDR_ewma, JR] = update_jamming_risk(p, alive, is_CH, PDR_ewma, JR, M, lambda);
    M_eff = max(M_min, round(M * (1 - JR)));

    for i = find(alive & ~is_CH)
        ch = CH_assign(i);
        if ch == 0; continue; end
        scale      = M_eff(i) / M;
        d_ch = sqrt((x(i) - x(ch))^2 + (y(i) - y(ch))^2);
        energy(i)  = energy(i)  - scale * compute_energy('tx', L, E_elec, E_amp, E_da, d_ch, 0);
        energy(ch) = energy(ch) - scale * compute_energy('rx', L, E_elec, E_amp, E_da, 0, 0);
    end

    total_recv = 0; total_sent = 0;
    CH_idx = find(is_CH & alive);
    for c = CH_idx
        mc = find(CH_assign == c & alive);
        nm = length(mc);
        if nm == 0; continue; end
        energy(c) = energy(c) - compute_energy('agg', L, E_elec, E_amp, E_da, 0, nm);
        m_eff_c = M_eff(mc);
        recv_count = 0;
        for mi = 1:nm
            recv_count = recv_count + sum(rand(m_eff_c(mi), 1) <= p(mc(mi)));
        end
        total_recv = total_recv + recv_count;
        total_sent = total_sent + sum(m_eff_c);
        d_bs = sqrt((x(c) - BS(1))^2 + (y(c) - BS(2))^2);
        energy(c) = energy(c) - compute_energy('tx', L, E_elec, E_amp, E_da, d_bs, 0);
    end

    n_stranded = sum(alive & ~is_CH & CH_assign == 0);
    total_sent = total_sent + sum(M_eff(alive & ~is_CH & CH_assign == 0));
    PDR_per_round(t) = (total_sent > 0) * total_recv / max(total_sent, 1);

    if PDR_per_round(t) == 0
        na = sum(alive);
        nc = length(CH_idx);
        nm = sum(alive & ~is_CH & CH_assign > 0);
        active_mem = alive & ~is_CH & CH_assign > 0;
        if nc == 0
            cause = 'CHs_ZERO';
            cause_counts.CHs_ZERO = cause_counts.CHs_ZERO + 1;
        elseif n_stranded == (na - nc)
            cause = 'ALL_STRANDED';
            cause_counts.ALL_STRANDED = cause_counts.ALL_STRANDED + 1;
        elseif any(active_mem) && mean(p(active_mem)) < 0.05
            cause = 'ALL_JAMMED';
            cause_counts.ALL_JAMMED = cause_counts.ALL_JAMMED + 1;
        else
            cause = sprintf('STOCHASTIC(mem=%d)', nm);
            cause_counts.STOCHASTIC = cause_counts.STOCHASTIC + 1;
        end
        fprintf('%-6d | %-5d | %-4d | %-8d | %-7d | %s\n', t, na, nc, n_stranded, nm, cause);
    end

    newly_dead         = alive & (energy <= 0);
    ch_died_last_round = any(is_CH & newly_dead);
    alive(newly_dead)  = false;
    if ~any(alive); break; end
end

fprintf('%s\n', repmat('-', 1, 60));
fprintf('Total zero-PDR: %d\n', sum(PDR_per_round == 0));
fprintf('  CHs_ZERO:     %d\n', cause_counts.CHs_ZERO);
fprintf('  ALL_STRANDED: %d\n', cause_counts.ALL_STRANDED);
fprintf('  ALL_JAMMED:   %d\n', cause_counts.ALL_JAMMED);
fprintf('  STOCHASTIC:   %d\n', cause_counts.STOCHASTIC);
fprintf('\nGeometry note: avg nearest-CH dist ~ %.1fm vs r_tx=%dm\n', ...
    sqrt(area^2 / round(p_CH * N)), r_tx);
