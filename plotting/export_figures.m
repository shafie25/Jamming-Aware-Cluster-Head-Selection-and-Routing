%% export_figures.m — Generate and save publication-quality figures for Overleaf
%
% Run this AFTER run_multiseed.m has finished (or it will re-run the sim).
% Saves PDF and PNG figures into figures/ for embedding in the LaTeX paper.
%
% Usage:
%   cd to project root, then: run plotting/export_figures.m
%
% Output files:
%   figures/fig_pdr.pdf           — PDR vs Round (3 schemes, mean ± std)
%   figures/fig_energy.pdf        — Residual Energy vs Round
%   figures/fig_alive.pdf         — Alive Nodes vs Round
%   figures/fig_combined.pdf      — 3-panel combined figure (for paper Fig. 2)

clc; clear; close all;
addpath(genpath('.'));

%% ---- Output directory ----
fig_dir = fullfile(pwd, 'figures');
if ~exist(fig_dir, 'dir'); mkdir(fig_dir); end

%% ---- Run simulation ----
fprintf('Running 20-seed simulation (Proposed vs TBC vs FCPA)...\n');

seeds     = 1:20;
n_seeds   = length(seeds);
n_schemes = 3;
config;

store = struct();
fields = {'PDR','energy','delay','alive'};
labels = {'proposed','tbc','fcpa'};
for f = 1:length(fields)
    for l = 1:length(labels)
        store.(fields{f}).(labels{l}) = zeros(n_seeds, T);
    end
end
tdeath = zeros(n_seeds, n_schemes);

for s = 1:n_seeds
    fprintf('  Seed %d/%d\n', s, n_seeds);
    rng(seeds(s));
    config; init_network; uav_trajectory;

    rp = run_proposed(x, y, BS, J_x, J_y, dist_to_BS, ...
        E0, d_max, T, K_elec, M, lambda, p_CH, r_c, r_exc, ...
        alpha, beta, gamma_, delta, phi1, phi2, phi3, ...
        p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
    store.PDR.proposed(s,:) = rp.PDR;
    store.energy.proposed(s,:) = rp.energy;
    store.delay.proposed(s,:) = rp.delay;
    store.alive.proposed(s,:) = rp.alive;
    tdeath(s,1) = rp.t_death;

    rt = run_tbc(x, y, BS, J_x, J_y, E0, T, M, p_base, kappa, r_j, E_elec, E_amp, L, r_tx);
    store.PDR.tbc(s,:) = rt.PDR;
    store.energy.tbc(s,:) = rt.energy;
    store.delay.tbc(s,:) = rt.delay;
    store.alive.tbc(s,:) = rt.alive;
    tdeath(s,2) = rt.t_death;

    rf = run_fcpa(x, y, BS, J_x, J_y, E0, T, M, p_CH, p_base, kappa, r_j, E_elec, E_amp, E_da, L, r_tx);
    store.PDR.fcpa(s,:) = rf.PDR;
    store.energy.fcpa(s,:) = rf.energy;
    store.delay.fcpa(s,:) = rf.delay;
    store.alive.fcpa(s,:) = rf.alive;
    tdeath(s,3) = rf.t_death;
end

scheme_names  = {'Proposed', 'TBC', 'FCPA'};
scheme_fields = labels;
ms = cell(1, n_schemes);
for k = 1:n_schemes
    fd = scheme_fields{k};
    ms{k}.PDR_mean    = mean(store.PDR.(fd),    1);
    ms{k}.PDR_std     = std( store.PDR.(fd),  0, 1);
    ms{k}.energy_mean = mean(store.energy.(fd), 1);
    ms{k}.energy_std  = std( store.energy.(fd),0, 1);
    ms{k}.delay_mean  = mean(store.delay.(fd),  1);
    ms{k}.delay_std   = std( store.delay.(fd), 0, 1);
    ms{k}.alive_mean  = mean(store.alive.(fd),  1);
    ms{k}.alive_std   = std( store.alive.(fd), 0, 1);
    ms{k}.label       = scheme_names{k};
end

fprintf('Simulation complete. Generating figures...\n');

%% ---- Style settings ----
rounds = 1:T;
colors = {
    [0.18, 0.45, 0.69], ...   % blue  — Proposed
    [0.80, 0.15, 0.15], ...   % red   — TBC
    [0.13, 0.63, 0.30]  ...   % green — FCPA
};
lw = 1.8;   % line width
fs_ax  = 9;   % axis label font size
fs_leg = 8;   % legend font size
fig_w  = 8.9; % cm — IEEE single column width
fig_h  = 6.5; % cm

%% ---- Figure helper ----
function save_fig(fig, fname, fig_dir)
    set(fig, 'Units', 'centimeters', 'Position', [2 2 8.9 6.5]);
    set(fig, 'PaperUnits', 'centimeters', 'PaperSize', [8.9 6.5]);
    pdf_path = fullfile(fig_dir, [fname '.pdf']);
    png_path = fullfile(fig_dir, [fname '.png']);
    exportgraphics(fig, pdf_path, 'ContentType', 'vector');
    exportgraphics(fig, png_path, 'Resolution', 300);
    fprintf('  Saved: %s\n', pdf_path);
end

%% ---- Fig A: PDR vs Round ----
fA = figure('Color','w');
ax = axes; hold(ax,'on');
for s = 1:n_schemes
    r = ms{s}; c = colors{s};
    upper = min(r.PDR_mean + r.PDR_std, 1);
    lower = max(r.PDR_mean - r.PDR_std, 0);
    fill([rounds, fliplr(rounds)], [upper, fliplr(lower)], ...
        c, 'FaceAlpha', 0.13, 'EdgeColor', 'none', 'HandleVisibility', 'off', 'Parent', ax);
    plot(ax, rounds, r.PDR_mean, 'Color', c, 'LineWidth', lw, 'DisplayName', r.label);
end
xlabel(ax, 'Round', 'FontSize', fs_ax);
ylabel(ax, 'PDR', 'FontSize', fs_ax);
title(ax, 'Packet Delivery Ratio', 'FontSize', fs_ax+1, 'FontWeight', 'bold');
legend(ax, 'Location', 'southwest', 'FontSize', fs_leg, 'Box', 'on');
grid(ax, 'on'); ylim(ax, [0 1]); xlim(ax, [1 T]);
set(ax, 'FontSize', fs_ax, 'TickLabelInterpreter', 'latex');
save_fig(fA, 'fig_pdr', fig_dir);

%% ---- Fig B: Residual Energy vs Round ----
fB = figure('Color','w');
ax = axes; hold(ax,'on');
for s = 1:n_schemes
    r = ms{s}; c = colors{s};
    upper = r.energy_mean + r.energy_std;
    lower = max(r.energy_mean - r.energy_std, 0);
    fill([rounds, fliplr(rounds)], [upper, fliplr(lower)], ...
        c, 'FaceAlpha', 0.13, 'EdgeColor', 'none', 'HandleVisibility', 'off', 'Parent', ax);
    plot(ax, rounds, r.energy_mean, 'Color', c, 'LineWidth', lw, 'DisplayName', r.label);
end
xlabel(ax, 'Round', 'FontSize', fs_ax);
ylabel(ax, 'Total Residual Energy (J)', 'FontSize', fs_ax);
title(ax, 'Network Energy', 'FontSize', fs_ax+1, 'FontWeight', 'bold');
legend(ax, 'Location', 'northeast', 'FontSize', fs_leg, 'Box', 'on');
grid(ax, 'on'); xlim(ax, [1 T]);
set(ax, 'FontSize', fs_ax);
save_fig(fB, 'fig_energy', fig_dir);

%% ---- Fig C: Alive Nodes vs Round ----
fC = figure('Color','w');
ax = axes; hold(ax,'on');
for s = 1:n_schemes
    r = ms{s}; c = colors{s};
    upper = min(r.alive_mean + r.alive_std, 100);
    lower = max(r.alive_mean - r.alive_std, 0);
    fill([rounds, fliplr(rounds)], [upper, fliplr(lower)], ...
        c, 'FaceAlpha', 0.13, 'EdgeColor', 'none', 'HandleVisibility', 'off', 'Parent', ax);
    plot(ax, rounds, r.alive_mean, 'Color', c, 'LineWidth', lw, 'DisplayName', r.label);
end
xlabel(ax, 'Round', 'FontSize', fs_ax);
ylabel(ax, 'Alive Nodes', 'FontSize', fs_ax);
title(ax, 'Network Lifetime', 'FontSize', fs_ax+1, 'FontWeight', 'bold');
legend(ax, 'Location', 'southwest', 'FontSize', fs_leg, 'Box', 'on');
grid(ax, 'on'); xlim(ax, [1 T]); ylim(ax, [0 100]);
set(ax, 'FontSize', fs_ax);
save_fig(fC, 'fig_alive', fig_dir);

%% ---- Fig D: 3-panel combined (for paper Fig. 2) ----
fD = figure('Color','w');
set(fD, 'Units', 'centimeters', 'Position', [2 2 18 5.5]);

metrics = {
    struct('field','PDR_mean', 'std_field','PDR_std', 'yl',[0 1],   'ylabel','PDR',                  'title','(a) Packet Delivery Ratio', 'leg','southwest'),
    struct('field','energy_mean','std_field','energy_std','yl',[0 50], 'ylabel','Residual Energy (J)', 'title','(b) Network Energy',         'leg','northeast'),
    struct('field','alive_mean','std_field','alive_std', 'yl',[0 100],'ylabel','Alive Nodes',          'title','(c) Network Lifetime',       'leg','southwest'),
};

for p = 1:3
    ax = subplot(1, 3, p); hold(ax, 'on');
    m  = metrics{p};
    for s = 1:n_schemes
        r = ms{s}; c = colors{s};
        mu  = r.(m.field);
        sig = r.(m.std_field);
        upper = mu + sig;  lower = mu - sig;
        if strcmp(m.field,'PDR_mean');    upper=min(upper,1);  lower=max(lower,0); end
        if strcmp(m.field,'alive_mean');  upper=min(upper,100);lower=max(lower,0); end
        lower = max(lower, 0);
        fill([rounds, fliplr(rounds)], [upper, fliplr(lower)], ...
            c, 'FaceAlpha', 0.13, 'EdgeColor', 'none', 'HandleVisibility', 'off', 'Parent', ax);
        plot(ax, rounds, mu, 'Color', c, 'LineWidth', lw, 'DisplayName', ms{s}.label);
    end
    xlabel(ax, 'Round', 'FontSize', fs_ax);
    ylabel(ax, m.ylabel, 'FontSize', fs_ax);
    title(ax, m.title, 'FontSize', fs_ax, 'FontWeight', 'bold');
    if p == 1
        legend(ax, 'Location', m.leg, 'FontSize', fs_leg, 'Box', 'on');
    end
    grid(ax, 'on'); xlim(ax, [1 T]); ylim(ax, m.yl);
    set(ax, 'FontSize', fs_ax);
end
set(fD, 'PaperUnits', 'centimeters', 'PaperSize', [18 5.5]);
pdf_path = fullfile(fig_dir, 'fig_combined.pdf');
png_path = fullfile(fig_dir, 'fig_combined.png');
exportgraphics(fD, pdf_path, 'ContentType', 'vector');
exportgraphics(fD, png_path, 'Resolution', 300);
fprintf('  Saved: %s\n', pdf_path);

fprintf('\nAll figures saved to: %s\n', fig_dir);
fprintf('Include in LaTeX with:\n');
fprintf('  \\includegraphics[width=\\linewidth]{figures/fig_combined.pdf}\n');
