%% export_figures.m — Generate and save publication-quality figures
%
% Run from project root after run_multiseed.m (or it will re-run the sim).
% Saves three separate PDF + PNG figures into figures/ for the LaTeX paper.
%
% Output files:
%   figures/fig_pdr.pdf       — Packet Delivery Ratio vs Round
%   figures/fig_energy.pdf    — Total Residual Energy vs Round
%   figures/fig_alive.pdf     — Alive Node Count vs Round

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
    ms{k}.alive_mean  = mean(store.alive.(fd),  1);
    ms{k}.alive_std   = std( store.alive.(fd), 0, 1);
    ms{k}.label       = scheme_names{k};
end

fprintf('Simulation complete. Generating figures...\n');

%% ---- Style constants ----
rounds = 1:T;
colors = {
    [0.18, 0.45, 0.69], ...   % blue  — Proposed
    [0.80, 0.15, 0.15], ...   % red   — TBC
    [0.13, 0.63, 0.30]  ...   % green — FCPA
};
lw     = 1.8;
fs_ax  = 10;
fs_leg = 9;
fig_w  = 8.9;   % cm — IEEE single-column width
fig_h  = 5.0;   % cm — reduced so 3 stacked figures fit on one page

%% ---- Helper: apply white/black style to axes ----
function style_axes(ax, fs_ax)
    set(ax, ...
        'Color',                'w', ...
        'XColor',               'k', ...
        'YColor',               'k', ...
        'GridColor',            [0.3 0.3 0.3], ...
        'GridAlpha',            0.25, ...
        'MinorGridColor',       [0.3 0.3 0.3], ...
        'FontSize',             fs_ax, ...
        'TickLabelInterpreter', 'latex', ...
        'LabelFontSizeMultiplier', 1.0);
    ax.XLabel.Color    = 'k';
    ax.YLabel.Color    = 'k';
    ax.Title.Color     = 'k';
    ax.Legend.Color    = 'w';
    ax.Legend.TextColor = 'k';
    ax.Legend.EdgeColor = 'k';
end

%% ---- Helper: save figure as PDF + PNG ----
function save_fig(fig, fname, fig_dir, fig_w, fig_h)
    set(fig, 'Color', 'w');
    set(fig, 'Units',      'centimeters', 'Position',  [2 2 fig_w fig_h]);
    set(fig, 'PaperUnits', 'centimeters', 'PaperSize', [fig_w fig_h]);
    pdf_path = fullfile(fig_dir, [fname '.pdf']);
    png_path = fullfile(fig_dir, [fname '.png']);
    exportgraphics(fig, pdf_path, 'ContentType', 'vector',  'BackgroundColor', 'white');
    exportgraphics(fig, png_path, 'Resolution',  300,        'BackgroundColor', 'white');
    fprintf('  Saved: %s\n', pdf_path);
end

%% ---- Fig 1: PDR vs Round ----
f1 = figure('Color', 'w');
ax = axes(f1); hold(ax, 'on');
for s = 1:n_schemes
    r = ms{s}; c = colors{s};
    upper = min(r.PDR_mean + r.PDR_std, 1);
    lower = max(r.PDR_mean - r.PDR_std, 0);
    fill([rounds, fliplr(rounds)], [upper, fliplr(lower)], ...
        c, 'FaceAlpha', 0.13, 'EdgeColor', 'none', ...
        'HandleVisibility', 'off', 'Parent', ax);
    plot(ax, rounds, r.PDR_mean, 'Color', c, 'LineWidth', lw, ...
        'DisplayName', r.label);
end
xlabel(ax, 'Round',  'Interpreter', 'latex');
ylabel(ax, 'PDR',    'Interpreter', 'latex');
title(ax,  'Packet Delivery Ratio ($\pm 1\sigma$)', ...
    'Interpreter', 'latex', 'FontWeight', 'bold');
legend(ax, 'Location', 'southwest', 'FontSize', fs_leg, 'Box', 'on');
grid(ax, 'on'); ylim(ax, [0 1]); xlim(ax, [1 T]);
style_axes(ax, fs_ax);
save_fig(f1, 'fig_pdr', fig_dir, fig_w, fig_h);

%% ---- Fig 2: Residual Energy vs Round ----
f2 = figure('Color', 'w');
ax = axes(f2); hold(ax, 'on');
for s = 1:n_schemes
    r = ms{s}; c = colors{s};
    upper = r.energy_mean + r.energy_std;
    lower = max(r.energy_mean - r.energy_std, 0);
    fill([rounds, fliplr(rounds)], [upper, fliplr(lower)], ...
        c, 'FaceAlpha', 0.13, 'EdgeColor', 'none', ...
        'HandleVisibility', 'off', 'Parent', ax);
    plot(ax, rounds, r.energy_mean, 'Color', c, 'LineWidth', lw, ...
        'DisplayName', r.label);
end
xlabel(ax, 'Round',                  'Interpreter', 'latex');
ylabel(ax, 'Total Residual Energy (J)', 'Interpreter', 'latex');
title(ax,  'Network Energy ($\pm 1\sigma$)', ...
    'Interpreter', 'latex', 'FontWeight', 'bold');
legend(ax, 'Location', 'northeast', 'FontSize', fs_leg, 'Box', 'on');
grid(ax, 'on'); xlim(ax, [1 T]);
style_axes(ax, fs_ax);
save_fig(f2, 'fig_energy', fig_dir, fig_w, fig_h);

%% ---- Fig 3: Alive Nodes vs Round ----
f3 = figure('Color', 'w');
ax = axes(f3); hold(ax, 'on');
for s = 1:n_schemes
    r = ms{s}; c = colors{s};
    upper = min(r.alive_mean + r.alive_std, 100);
    lower = max(r.alive_mean - r.alive_std, 0);
    fill([rounds, fliplr(rounds)], [upper, fliplr(lower)], ...
        c, 'FaceAlpha', 0.13, 'EdgeColor', 'none', ...
        'HandleVisibility', 'off', 'Parent', ax);
    plot(ax, rounds, r.alive_mean, 'Color', c, 'LineWidth', lw, ...
        'DisplayName', r.label);
end
xlabel(ax, 'Round',        'Interpreter', 'latex');
ylabel(ax, 'Alive Nodes',  'Interpreter', 'latex');
title(ax,  'Network Lifetime ($\pm 1\sigma$)', ...
    'Interpreter', 'latex', 'FontWeight', 'bold');
legend(ax, 'Location', 'southwest', 'FontSize', fs_leg, 'Box', 'on');
grid(ax, 'on'); xlim(ax, [1 T]); ylim(ax, [0 100]);
style_axes(ax, fs_ax);
save_fig(f3, 'fig_alive', fig_dir, fig_w, fig_h);

fprintf('\nAll figures saved to: %s\n', fig_dir);
