# Jacobian Path Planning Example

This MATLAB/Simulink project demonstrates Jacobian-based path planning with selectable robot models and controller algorithms. The model uses fixed three-element interfaces, so changing a selection does not require rewiring the diagram.

## Files

- `JacobianAlgorithms.prj` - MATLAB project file.
- `jacobian_path_planning.slx` - top-level Simulink model.
- `create_path_planning_model.m` - recreates the model and its selectable blocks.
- `setup_project.m` - adds the folder to the MATLAB path and initializes defaults.
- `run_example.m` - runs the default configuration and plots the result.

## Requirements

- MATLAB R2024a or newer
- Simulink
- MATLAB Function blocks

## Quick start

```matlab
setup_project
run_example
```

The default configuration is:

| Selection | Value | Meaning |
|---|---:|---|
| `algorithm_selection` | `1` | Damped Jacobian |

Both selectors are visible in `jacobian_path_planning.slx` as the **Model Selection** and **Algorithm Selection** Constant blocks. Change their values to `1` or `2`, update the diagram, and run the simulation:

```matlab
algorithm_selection = 2;
target = [0.9; 0.4; 0.8];
sim('jacobian_path_planning')
```

The corresponding `Multiport Switch` blocks select the active implementation:

- **Algorithm Selector** selects the planner.

## Model choices

### Model 1 - two-link planar arm

![two-link planar arm](img/IMG_1144.PNG "two-link planar arm")


The arm uses link lengths `L1` and `L2`:

```text
x = L1*cos(q1) + L2*cos(q1 + q2)
y = L1*sin(q1) + L2*sin(q1 + q2)
z = 0
```

The fixed three-element state is `[q1; q2; 0]`. Only the first two target coordinates are used.

## Proposed reachable targets

Use `workspace_targets(model_selection)` to return a list of targets. The
rows are `[x, y, z]`; the z-coordinate is zero for the planar arm.

- Two-link arm: `[1.25; 0.65]`, `[0.95; -0.75]`, `[0.25; 1.20]`

## Algorithm choices

### Algorithm 1 - damped Jacobian

The planner uses damped least squares:

```text
q_dot = J' * (J*J' + lambda^2*I)^(-1) * Kp*e
```

with `lambda = 0.05`. Damping improves behavior near singular configurations.

### Algorithm 2 - pseudoinverse transposition tradeoff

The planner uses a λ-scaled Jacobian-transpose/pseudoinverse tradeoff:

```text
q_dot = J' * (lambda*(J*J') + (1-lambda)*I)^(-1) * Kp*e
```

The implementation uses `lambda = 0.05`. At `lambda = 1` this approaches the
Moore-Penrose solution of `J q_dot = Kp*e`; at `lambda = 0` it becomes the
Jacobian-transpose update.

Both algorithms apply component-wise joint-speed saturation using `max_joint_speed`.

## Workspace variables

```matlab
L1 = 1.0;
L2 = 0.8;
initial_q = [0; 0; 0];
target = [1.25; 0.65; 0.2];
Kp = 2.0;
simulation_time = 8.0;
algorithm_selection = 1;
```

The model logs `q_log` and `end_effector_log` to the simulation output. `run_example(false)` runs without creating plots, which is useful for automated tests.
