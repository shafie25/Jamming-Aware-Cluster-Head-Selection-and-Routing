%% plot_multiseed.m — Multi-Seed Results Plot (mean ± std)
% Plots PDR, energy, delay, and alive nodes for all schemes.
% Each metric shows a solid mean line with a shaded ±1 std band.
% Supports up to 5 schemes.
%
% Inputs:
%   results_all — cell array of structs, one per scheme.
%                 Each struct: PDR_mean, PDR_std, energy_mean, energy_std,
%                 delay_mean, delay_std, alive_mean, alive_std, label
%   T           — total simulation rounds

function plot_multiseed(results_all, T)

    rounds    = 1:T;
    n_schemes = length(results_all);

    % Colorblind-friendly palette (blue, red, green, orange, purple)
    colors = {
        [0.18, 0.45, 0.69], ...   % blue   — Proposed
        [0.80, 0.15, 0.15], ...   % red    — LEACH
        [0.20, 0.63, 0.17], ...   % green  — EWMA-Detect
        [0.89, 0.47, 0.10], ...   % orange — Threshold-JR
        [0.56, 0.12, 0.71]  ...   % purple — Reactive-CH
    };

    figure('Position', [100, 100, 1400, 900]);

    %% Panel 1 — Packet Delivery Ratio
    subplot(2, 2, 1); hold on;
    for s = 1:n_schemes
        r = results_all{s};
        c = colors{s};
        upper = min(r.PDR_mean + r.PDR_std, 1);
        lower = max(r.PDR_mean - r.PDR_std, 0);
        fill([rounds, fliplr(rounds)], [upper, fliplr(lower)], ...
            c, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(rounds, r.PDR_mean, 'Color', c, 'LineWidth', 1.5, 'DisplayName', r.label);
    end
    xlabel('Round'); ylabel('PDR');
    title('Packet Delivery Ratio (mean \pm std)');
    legend('Location', 'southwest', 'FontSize', 8); grid on; ylim([0 1]);

    %% Panel 2 — Total Residual Energy
    subplot(2, 2, 2); hold on;
    for s = 1:n_schemes
        r = results_all{s};
        c = colors{s};
        upper = r.energy_mean + r.energy_std;
        lower = max(r.energy_mean - r.energy_std, 0);
        fill([rounds, fliplr(rounds)], [upper, fliplr(lower)], ...
            c, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(rounds, r.energy_mean, 'Color', c, 'LineWidth', 1.5, 'DisplayName', r.label);
    end
    xlabel('Round'); ylabel('Total Residual Energy (J)');
    title('Network Energy (mean \pm std)');
    legend('Location', 'northeast', 'FontSize', 8); grid on;

    %% Panel 3 — Average End-to-End Delay
    subplot(2, 2, 3); hold on;
    for s = 1:n_schemes
        r = results_all{s};
        c = colors{s};
        upper = r.delay_mean + r.delay_std;
        lower = max(r.delay_mean - r.delay_std, 0);
        fill([rounds, fliplr(rounds)], [upper, fliplr(lower)], ...
            c, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(rounds, r.delay_mean, 'Color', c, 'LineWidth', 1.5, 'DisplayName', r.label);
    end
    xlabel('Round'); ylabel('Avg Hops');
    title('Average End-to-End Delay (mean \pm std)');
    legend('Location', 'northeast', 'FontSize', 8); grid on;

    %% Panel 4 — Alive Nodes
    subplot(2, 2, 4); hold on;
    for s = 1:n_schemes
        r = results_all{s};
        c = colors{s};
        upper = min(r.alive_mean + r.alive_std, 100);
        lower = max(r.alive_mean - r.alive_std,   0);
        fill([rounds, fliplr(rounds)], [upper, fliplr(lower)], ...
            c, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(rounds, r.alive_mean, 'Color', c, 'LineWidth', 1.5, 'DisplayName', r.label);
    end
    xlabel('Round'); ylabel('Alive Nodes');
    title('Network Lifetime (mean \pm std)');
    legend('Location', 'southwest', 'FontSize', 8); grid on;

    sgtitle('Jamming-Aware WSN Simulation — 5-Seed Average (All Schemes)');

end
