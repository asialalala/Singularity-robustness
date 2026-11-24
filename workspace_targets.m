function targets = workspace_targets(modelSelection)
%WORKSPACE_TARGETS Return reachable example targets for each robot model.
%
% targets = workspace_targets(1) for the two-link planar arm.
% targets = workspace_targets(2) for the three-joint manipulator.

if nargin < 1
    modelSelection = 1;
end

if modelSelection == 1
    targets = [
        1.25  0.65  0.00
        0.95 -0.75  0.00
        0.25  1.20  0.00
    ];
elseif modelSelection == 2
    targets = [
        0.95  0.45  0.55
       -0.70  0.80  0.85
        0.30 -0.90  0.30
    ];
else
    error('modelSelection must be 1 or 2.');
end
end
