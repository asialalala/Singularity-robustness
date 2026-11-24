function results = run_example(makePlot)
%RUN_EXAMPLE Simulate the Jacobian path-planning example and plot results.

if nargin == 0
    makePlot = true;
end

setup_project;

model = 'jacobian_path_planning';
load_system(model);
results = sim(model, 'ReturnWorkspaceOutputs', 'on');

target = evalin('base', 'target');
q = results.q_log;
position = results.end_effector_log;
qValues = squeeze(q.signals.values);
positionValues = squeeze(position.signals.values);
if size(qValues, 2) == numel(q.time)
    qValues = qValues.';
end
if size(positionValues, 2) == numel(position.time)
    positionValues = positionValues.';
end

if makePlot
    figure('Name', 'Jacobian Path Planning Example', 'Color', 'w');
    tiledlayout(1, 2);

    nexttile;
  
        plot(positionValues(:, 1), positionValues(:, 2), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(positionValues(1, 1), positionValues(1, 2), 'ko', 'MarkerFaceColor', 'k');
        plot(target(1), target(2), 'rx', 'LineWidth', 2, 'MarkerSize', 10);
        axis equal;
        grid on;
        xlabel('x [m]');
        ylabel('y [m]');
        title('End-effector path');
        legend('Path', 'Start', 'Target', 'Location', 'best');

    nexttile;
    plot(q.time, qValues, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Joint angle [rad]');
    title('Joint states');
    legend('q_1', 'q_2', 'Location', 'best');

end

fprintf('Final end-effector position: [%.4f, %.4f] m\n', ...
    positionValues(end, 1), positionValues(end, 2));

    fprintf('Target position:            [%.4f, %.4f] m\n', target(1), target(2));
end
