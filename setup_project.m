function setup_project()
%SETUP_PROJECT Prepare the Jacobian path-planning example.

projectFolder = fileparts(mfilename('fullpath'));
addpath(projectFolder);

assignin('base', 'L1', 1.0);
assignin('base', 'L2', 0.8);
assignin('base', 'a0', 0.2);
assignin('base', 'a1', 0.6);
assignin('base', 'a2', 0.5);
assignin('base', 'a3', 0.35);
assignin('base', 'initial_q', [0; 0; 0]);
assignin('base', 'target', [1.25; 0.65; 0.2]);
assignin('base', 'Kp', 2.0);
assignin('base', 'max_joint_speed', 1.5);
assignin('base', 'simulation_time', 8.0);
assignin('base', 'model_selection', 1);     % Select model here
assignin('base', 'algorithm_selection', 2); % Select algorithm here

% Validate targets against the exact workspace limits used by this example.
modelSelection = evalin('base', 'model_selection');
configuredTarget = evalin('base', 'target');
if modelSelection == 1
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
else
    a0 = evalin('base', 'a0');
    a1 = evalin('base', 'a1');
    a2 = evalin('base', 'a2');
    a3 = evalin('base', 'a3');
    radialLimit = a1+a2+a3;
    targetRadial = norm(configuredTarget(1:2));
    targetZ = configuredTarget(3)-a0;
    reachable = targetRadial <= radialLimit + eps && ...
        abs(targetZ) <= abs(a2+a3)+eps;
    if ~reachable
        warning('Configured target appears outside the three-joint workspace.');
    end
end

modelFile = fullfile(projectFolder, 'jacobian_path_planning.slx');
if ~isfile(modelFile)
    create_path_planning_model;
end

fprintf('Jacobian path-planning example is ready.\n');
fprintf('Run "run_example" to simulate and plot the result.\n');
end
