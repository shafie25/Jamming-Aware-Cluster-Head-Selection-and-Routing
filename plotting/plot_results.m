%% plot_results.m — Simulation Results Plotting
% Plots PDR, total energy, average delay, and alive nodes over rounds
% for all schemes passed in. Each scheme is a results struct from run_*.m
%
% Inputs:
%   results_all — cell array of results structs, one per scheme
%   T           — total simulation rounds

function plot_results(results_all, T)

    rounds = 1:T;
    % Colorblind-friendly palette matching plot_multiseed.m
    colors = {
        [0.18, 0.45, 0.69], ...   % blue   — Proposed
        [0.80, 0.15, 0.15], ...   % red    — LEACH
        [0.20, 0.63, 0.17], ...   % green  — EWMA-Detect
        [0.89, 0.47, 0.10], ...   % orange — Threshold-JR
        [0.56, 0.12, 0.71]  ...   % purple — Reactive-CH
    };
    n_schemes = length(results_all);

    figure('Position', [100, 100, 1200, 800]);

    %% Plot 1 — Packet Delivery Ratio
    subplot(2, 2, 1);
    hold on;
    for s = 1:n_schemes
        r = results_all{s};
        plot(rounds, r.PDR, colors{s}, 'LineWidth', 1.5, 'DisplayName', r.label);
    end
    xlabel('Round'); ylabel('PDR');
    title('Packet Delivery Ratio vs. Round');
    legend('Location', 'southwest'); grid on; ylim([0 1]);

    %% Plot 2 — Total Residual Energy
    subplot(2, 2, 2);
    hold on;
    for s = 1:n_schemes
        r = results_all{s};
        plot(rounds, r.energy, colors{s}, 'LineWidth', 1.5, 'DisplayName', r.label);
    end
    xlabel('Round'); ylabel('Total Residual Energy (J)');
    title('Network Energy vs. Round');
    legend('Location', 'northeast'); grid on;

    %% Plot 3 — Average End-to-End Delay (hop count)
    subplot(2, 2, 3);
    hold on;
    for s = 1:n_schemes
        r = results_all{s};
        plot(rounds, r.delay, colors{s}, 'LineWidth', 1.5, 'DisplayName', r.label);
    end
    xlabel('Round'); ylabel('Avg Hops');
    title('Average End-to-End Delay vs. Round');
    legend('Location', 'northeast'); grid on;

    %% Plot 4 — Alive Nodes
    subplot(2, 2, 4);
    hold on;
    for s = 1:n_schemes
        r = results_all{s};
        plot(rounds, r.alive, colors{s}, 'LineWidth', 1.5, 'DisplayName', r.label);
        % Mark first node death
        if ~isnan(r.t_death)
            xline(r.t_death, '--', colors{s}, 'Alpha', 0.5);
        end
    end
    xlabel('Round'); ylabel('Alive Nodes');
    title('Network Lifetime vs. Round');
    legend('Location', 'southwest'); grid on;

    sgtitle('Jamming-Aware WSN Simulation Results');

end