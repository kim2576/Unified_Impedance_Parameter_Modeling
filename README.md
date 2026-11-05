# Mode-Unified Impedance Parameterization

MATLAB implementation of the stance-phase impedance parameterization described in Section III and Appendix B of the associated manuscript. The code converts RBF-unified knee and ankle biomechanics into terrain-slope-dependent stiffness (`K`), damping (`B`), and equilibrium-angle (`theta_eq`) trajectories over -35° to +35°.

## Method

For each terrain slope,

```text
tau = -K (theta - theta_eq) - B theta_dot
```

`theta`, `theta_dot`, and the target joint moment are taken from the continuous biomechanics model. `K` and `B` are fourth-order polynomials of normalized stance phase and are identified separately over early stance (ES) and late stance (LS) using constrained linear least squares.

The main processing sequence is:

1. load the RBF-unified knee/ankle angle, angular-velocity, and moment surfaces and the discrete able-bodied (AB) reference moments;
2. construct the knee and ankle equilibrium-angle trajectories;
3. define slope-dependent stiffness and damping bounds;
4. identify ES and LS stiffness/damping polynomials;
5. assemble full-stance impedance and reconstructed-moment surfaces;
6. evaluate reconstructed moments against the AB reference trajectories; and
7. export 101 × 71 phase-by-slope matrices for experimental comparison.

## Quick start

Run from the repository root:

```matlab
evaluate_and_plot
```

Joint-specific settings are in:

```text
src/config/get_joint_config.m
```

The angular-velocity input is selected near the top of `evaluate_and_plot.m`:

```matlab
angular_velocity_variant = "original"; % or "norm"
```

Both choices are loaded from `RBF_model_full`; angular velocity is not recomputed from the angle surface in this repository.

## Input data

The `data/` directory contains:

```text
data/
├── RBF_models.mat
└── AB_refs.mat
```

`RBF_models.mat` contains `RBF_model_full`, and `AB_refs.mat` contains `target_biomechanics_total`.

Required angular-velocity fields are:

```text
knee_angle_dot_stance
ankle_angle_dot_stance
knee_angle_dot_stance_norm
ankle_angle_dot_stance_norm
```

RBF surfaces use phase × slope orientation (101 × 71). The optimizer uses the first 100 stance samples and works internally in slope × phase orientation (71 × 100).

## Equilibrium angle

The stance equilibrium angle is organized into three functional regions:

- **R1 — weight acceptance:** transition from heel contact toward the support reference;
- **R2 — single support:** hold the support reference; and
- **R3 — unloading:** transition toward the terminal stance/toe-off reference.

The support reference is selected from the joint angle associated with the mid-to-late stance minimum of the biological moment.

For the knee, level walking and ascent use the three-region structure directly. Descent uses a slope-dependent proportional-following term during weight acceptance before convergence to the support reference.

For the ankle, the support reference is capped at 10° dorsiflexion. `follow_gain` controls proportional yielding, and `push_off_duration` sets the R3 transition length.

## Stiffness and damping

For terrain slope `s` and normalized stance phase `p`,

```text
K(s,p) = k4 p^4 + k3 p^3 + k2 p^2 + k1 p + k0
B(s,p) = b4 p^4 + b3 p^3 + b2 p^2 + b1 p + b0
```

ES is identified first, and its terminal stiffness and damping initialize LS. The slope-dependent upper bounds `K_max(s)` and `B_max(s)` are arctangent functions fitted through the steep-descent, level-walking, and steep-ascent reference values. Constant lower bounds define `K_min` and `B_min`.

The constrained fit enforces the ES/LS boundary conditions, impedance bounds, and the derivative limits listed in Appendix B of the manuscript. The phase references in `get_joint_config.m` are offline identification settings, not real-time FSM transition times.

## Configuration

The main settings are centralized in `src/config/get_joint_config.m`.

| Field | Description |
| --- | --- |
| `phase.ES_weight_peak` | ES loading/support reference phase. |
| `phase.ES_end` | Last ES sample. |
| `phase.LS_start` | First LS sample. |
| `phase.LS_peak_target` | LS stiffness-reference phase `p_K`. |
| `bounds.K.max`, `bounds.B.max` | Maximum arctangent-envelope values. |
| `bounds.*.upper_min_ratio` | Minimum upper-envelope value relative to its maximum. |
| `bounds.*.lower_ratio` | Constant lower bound relative to the maximum. |
| `ES.derivative.*` | ES derivative constraints. |
| `LS.derivative.*` | LS derivative constraints. |
| `equilibrium.*` | Joint-specific support-angle and transition settings. |
| `solver.constraint_tolerance` | Feasibility diagnostic tolerance. |

The default values correspond to the manuscript configuration.

## Evaluation and outputs

`evaluate_impedance_fit.m` compares reconstructed moments with the discrete AB reference trajectories. Reported metrics are:

- MAE over full stance, ES, and LS;
- phase-aligned Pearson r² over full stance, ES, and LS; and
- peak-moment MAE across AB reference slopes.

Phase alignment is performed independently within each interval using normalized cross-correlation before Pearson r² is computed. Peak moment uses the full stance phase for level walking/ascent and samples 40–100 for descent.

Running `evaluate_and_plot` writes evaluation summaries under `outputs/Evaluation/` and creates:

```text
outputs/Impedance_model_full.mat
```

`impedance_model_full` contains:

```text
knee_K_stance
knee_B_stance
knee_theta_eq_stance
knee_reconstructed_moment_stance
ankle_K_stance
ankle_B_stance
ankle_theta_eq_stance
ankle_reconstructed_moment_stance
slope
stance_phase
```

The impedance surfaces are exported as 101 × 71 phase-by-slope matrices.

## Figures

For each joint, the script generates reconstructed moment, stiffness, damping, and equilibrium-angle surfaces. The stiffness and damping figures include the slope-dependent arctangent upper bounds.

## Repository structure

```text
.
├── evaluate_and_plot.m
├── README.md
├── data/
│   ├── AB_refs.mat
│   ├── RBF_models.mat
│   └── README.md
└── src/
    ├── bounds/
    ├── config/
    ├── equilibrium/
    ├── evaluation/
    ├── io/
    ├── optimization/
    └── visualization/
```

`outputs/` is created when the main script is run.

## MATLAB requirements

- MATLAB
- Optimization Toolbox (`lsqlin`, `lsqcurvefit`)
- Signal Processing Toolbox (`xcorr`)

## Upstream biomechanics model

The input RBF surfaces are generated by the continuous locomotion-biomechanics modeling repository. This repository starts from those surfaces and performs the impedance parameterization.

## Citation

If you use this code in academic work, please cite the associated publication describing the mode-unified biomechanics and impedance-parameterization method.
