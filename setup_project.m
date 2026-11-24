function setup_project()
%SETUP_PROJECT Prepare the Jacobian path-planning example.

projectFolder = fileparts(mfilename('fullpath'));
addpath(projectFolder);

assignin('base', 'L1', 1.0);
assignin('base', 'L2', 0.8);
assignin('base', 'initial_q', [0; 0]);
assignin('base', 'target', [1.25; 0.65]);
assignin('base', 'Kp', 2.0);
assignin('base', 'simulation_time', 8.0);
assignin('base', 'algorithm_selection', 1); % Select algorithm here

% Validate targets against the exact workspace limits used by this example.
configuredTarget = evalin('base', 'target');
    L1 = evalin('base', 'L1');
    L2 = evalin('base', 'L2');
    innerRadius = abs(L1-L2);
    outerRadius = L1+L2;
    targetRadius = norm(configuredTarget(1:2));
    reachable = targetRadius >= innerRadius && targetRadius <= outerRadius;
    if ~reachable
        warning('Configured target is outside the two-link workspace annulus [%.3g, %.3g] m.', ...
            innerRadius, outerRadius);
    end


modelFile = fullfile(projectFolder, 'jacobian_path_planning.slx');
if ~isfile(modelFile)
    create_path_planning_model;
end

fprintf('Jacobian path-planning example is ready.\n');
fprintf('Run "run_example" to simulate and plot the result.\n');
end
