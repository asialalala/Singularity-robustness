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
assignin('base', 'model_selection', 1);
assignin('base', 'algorithm_selection', 1);

modelFile = fullfile(projectFolder, 'jacobian_path_planning.slx');
if ~isfile(modelFile)
    create_path_planning_model;
end

fprintf('Jacobian path-planning example is ready.\n');
fprintf('Run "run_example" to simulate and plot the result.\n');
end
