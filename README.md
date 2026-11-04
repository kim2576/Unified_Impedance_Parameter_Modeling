# Mode-Unified Impedance Parameterization

MATLAB framework for transforming continuous knee and ankle biomechanics into slope- and stance-phase-dependent impedance parameters for robotic lower-limb prosthesis control.

This repository implements the stance-phase parameterization described in Section III and Appendix B of the associated manuscript. RBF-unified joint kinematics and kinetics are converted into continuous stiffness (`K`), damping (`B`), and equilibrium-angle (`theta_eq`) trajectories across terrain slopes from -35° to +35°.

## Method overview

For each terrain slope, the impedance relationship is written as

```text
tau = -K (theta - theta_eq) - B theta_dot
```

where `theta`, `theta_dot`, and the target joint moment are obtained from the continuous biomechanics model. `K` and `B` are represented as fourth-order polynomials of normalized stance phase and identified independently over early stance (ES) and late stance (LS) with constrained linear least squares.

The workflow:

1. loads RBF-unified knee/ankle angle, angular-velocity, and moment surfaces plus discrete able-bodied (AB) reference moments;
2. constructs joint-specific equilibrium-angle trajectories from the stance angle-moment relationship;
3. fits continuous slope-dependent upper envelopes for stiffness and damping using arctangent functions;
4. identifies ES and LS stiffness/damping polynomials subject to equality, bound, and derivative-shape constraints;
5. assembles continuous full-stance impedance and reconstructed-moment surfaces;
6. evaluates reconstructed moments against AB reference trajectories using MAE, peak MAE, and phase-aligned Pearson r²; and
7. exports 101 × 71 phase-by-slope matrices for later comparison with experimental data.

## Quick start

From the repository root, run:

```matlab
evaluate_and_plot
```

The main script processes knee and ankle independently using the configuration in:

```text
src/config/get_joint_config.m
```

At the top of `evaluate_and_plot.m`, select the RBF-unified angular-velocity input:

```matlab
angular_velocity_variant = "original"; % or "norm"
```

Both options are loaded directly from `RBF_model_full`; angular velocity is treated as an independently modeled biomechanical input and is not recomputed inside this repository.

## Inputs

The `data/` directory contains:

```text
data/
├── RBF_models.mat
└── AB_refs.mat
```

`RBF_models.mat` must contain `RBF_model_full`, and `AB_refs.mat` must contain `target_biomechanics_total`.

The stance angular-velocity fields used by this package are:

```text
knee_angle_dot_stance
ankle_angle_dot_stance
knee_angle_dot_stance_norm
ankle_angle_dot_stance_norm
```

Only the pair selected by `angular_velocity_variant` is required for a given run (`original` or `norm`).

Each RBF surface is expected in phase × slope orientation (101 × 71). The optimizer uses the first 100 stance samples internally and transposes them to slope × phase orientation (71 × 100).

## Equilibrium-angle parameterization

The stance equilibrium angle follows the three functional regions described in the manuscript:

- **R1 — weight acceptance:** `theta_eq` transitions from heel contact toward a support angle near the first vGRF peak;
- **R2 — single support:** `theta_eq` remains at the support reference so joint displacement generates stance support through impedance; and
- **R3 — unloading:** `theta_eq` transitions toward the terminal stance/toe-off reference to facilitate ankle push-off and knee swing initiation.

The support angle is selected from the joint angle associated with the mid-to-late stance minimum of the biological moment.

### Knee

For level walking and ascent, the knee follows the three-region structure directly. During descent, a slope-dependent proportional-following strategy allows controlled knee flexion during weight acceptance before `theta_eq` converges to the support reference.

### Ankle

The ankle support angle is capped at 10° dorsiflexion to remain compatible with prosthesis range of motion and preserve plantarflexion torque capacity. During yielding, `follow_gain` controls how closely `theta_eq` follows the measured ankle angle before converging to the support reference. During R3, `push_off_duration` determines the number of phase samples used to transition from the support reference toward the terminal ankle equilibrium angle.

## Stiffness and damping parameterization

For each terrain slope `s`, stiffness and damping are fourth-order polynomials of normalized stance phase `p`:

```text
K(s,p) = k4 p^4 + k3 p^3 + k2 p^2 + k1 p + k0
B(s,p) = b4 p^4 + b3 p^3 + b2 p^2 + b1 p + b0
```

The ES coefficients are identified first. Their terminal values initialize the LS optimization, maintaining continuity between the two intervals.

### Slope-dependent bounds

The upper envelopes `K_max(s)` and `B_max(s)` are represented by continuous arctangent functions fitted through reference values at steep descent (-35°), level walking (0°), and steep ascent (+35°).

- `K_max(s)` increases with terrain slope to accommodate greater support demands during ascent.
- `B_max(s)` decreases with terrain slope to allow greater energy dissipation during descent.
- `K_min` and `B_min` are constant lower bounds and define the minimum impedance values at heel contact.

### Equality and shape constraints

The constrained LSQ formulation implements the manuscript design principles:

- ES begins at `K_min` and `B_min` at heel contact.
- LS is initialized from the terminal ES stiffness and damping values.
- `LS_peak_target` defines the stiffness-reference phase `p_K` near 75% stance, preserving support-level stiffness before unloading.
- derivative limits regulate increasing impedance during loading, approximately constant behavior through support, and constant-or-decreasing behavior during unloading.

The optimization phase references are offline parameter-identification settings; they are not the real-time FSM transition timing.

## Configuration parameters

Joint-specific settings are centralized in `src/config/get_joint_config.m`.

| Configuration field | Role in the parameterization |
| --- | --- |
| `phase.ES_weight_peak` | Reference phase near the first vGRF peak used in the ES loading-shape constraints. |
| `phase.ES_end` | Terminal phase sample included in the ES polynomial optimization. |
| `phase.LS_start` | First phase sample included in the LS optimization; must immediately follow `ES_end`. |
| `phase.LS_peak_target` | LS stiffness-reference phase `p_K`, approximately 75% stance. |
| `bounds.K.max` | Maximum stiffness upper-envelope value used by the slope-dependent arctangent bound. |
| `bounds.B.max` | Maximum damping upper-envelope value used by the slope-dependent arctangent bound. |
| `bounds.*.upper_min_ratio` | Sets the minimum value of each slope-dependent upper envelope relative to its maximum. |
| `bounds.*.lower_ratio` | Sets the constant lower bound relative to the corresponding maximum bound. |
| `ES.derivative.*` | Signed derivative limits governing K/B progression during ES loading and support. |
| `LS.derivative.K.pre_peak_mode` | Applies the LS1 stiffness derivative lower bound throughout LS1 (`interval`) or only at `p_K` (`peak_only`). |
| `LS.derivative.K.pre_peak_min` | Minimum stiffness derivative permitted during LS1 before the unloading reference phase. |
| `LS.derivative.K.post_peak_max` | Maximum stiffness derivative permitted during LS2 unloading. |
| `LS.derivative.B.post_peak_max` | Maximum damping derivative permitted during LS2 unloading. |
| `equilibrium.moment_search_start/end` | Phase window used to locate the mid-to-late stance moment minimum that defines the support angle. |
| `equilibrium.flexion_delay` | Optional sample offset applied after locating the knee support-angle reference. |
| `equilibrium.descent_follow_start/end` | Endpoints of the slope-dependent knee proportional-following factor during descent. |
| `equilibrium.minimum_support_angle` | Lower limit applied to the knee support-angle reference. |
| `equilibrium.follow_gain` | Ankle proportional-yielding factor used while `theta_eq` follows measured ankle angle. |
| `equilibrium.support_start_phase/support_search_end` | Ankle moment-search window used to select the support reference. |
| `equilibrium.max_support_angle` | Maximum ankle dorsiflexion support reference. |
| `equilibrium.push_off_duration` | Number of samples used for the ankle R3 transition toward its terminal equilibrium angle. |
| `equilibrium.max_terminal_angle` | Upper limit on the terminal ankle equilibrium-angle reference. |
| `solver.constraint_tolerance` | Numerical tolerance used only to report constraint-feasibility diagnostics. |

The default configuration uses the joint-specific ES/LS phase boundaries and derivative limits listed in the manuscript Appendix B/Table A1.

## Experimenting with the configuration

`src/config/get_joint_config.m` is intended to make the main design choices easy to inspect and modify. The values in the default configuration reproduce the parameterization used in the manuscript; changes below are examples for sensitivity checks or controller-design exploration.

A useful workflow is:

1. run the default configuration once with `evaluate_and_plot`;
2. save or rename the generated `outputs/` directory;
3. change one parameter at a time in `get_joint_config.m`;
4. rerun `evaluate_and_plot`; and
5. compare the impedance surfaces, reconstructed moments, MAE/peak-MAE tables, and phase-aligned Pearson r².

For example, after a baseline run:

```matlab
evaluate_and_plot
movefile("outputs", "outputs_baseline")
```

edit one configuration value, then run again:

```matlab
evaluate_and_plot
```

The new results will be written to a fresh `outputs/` directory.

### Example 1 — ankle support-angle limit

Default:

```matlab
cfg.equilibrium.max_support_angle = 10;
```

Trial:

```matlab
cfg.equilibrium.max_support_angle = 8;
```

This tests a more restrictive dorsiflexion support reference. Inspect:

```text
outputs/Plots/ankle_equilibrium_angle.png
outputs/Plots/ankle_reconstructed_moment.png
outputs/Evaluation/reconstruction_peak_mae_summary.csv
outputs/Evaluation/reconstruction_r2_summary.csv
```

### Example 2 — ankle proportional yielding

Default:

```matlab
cfg.equilibrium.follow_gain = 1.1;
```

Trial:

```matlab
cfg.equilibrium.follow_gain = 1.0;
```

`follow_gain` controls the proportional relation between the measured ankle angle and the support reference during R1 yielding. A value of `1.0` makes the prescribed equilibrium angle follow the measured ankle angle directly in that relation, while the default `1.1` introduces a larger offset relative to the support reference. Compare the early-stance equilibrium-angle and reconstructed-moment surfaces.

### Example 3 — stiffness upper envelope

Default:

```matlab
cfg.bounds.K.max = 0.08;
```

Trial:

```matlab
cfg.bounds.K.max = 0.07;
```

This lowers the maximum value available to the slope-dependent stiffness envelope. Compare:

```text
outputs/Plots/knee_stiffness_K.png
outputs/Plots/ankle_stiffness_K.png
outputs/Plots/knee_K_upper_bound_arctan.png
outputs/Plots/ankle_K_upper_bound_arctan.png
```

and check whether reconstruction MAE or phase-aligned r² changes at the steeper slopes.

### Example 4 — late-stance stiffness-shape constraint

For either joint, change:

```matlab
cfg.LS.derivative.K.pre_peak_min = -0.005;
```

to a tighter value such as:

```matlab
cfg.LS.derivative.K.pre_peak_min = -0.001;
```

This more strongly limits how quickly stiffness can decrease between LS onset and the stiffness-reference phase `p_K`. Compare the resulting stiffness plateau and reconstructed moment before unloading.

The constraint can also be localized to the reference phase by changing:

```matlab
cfg.LS.derivative.K.pre_peak_mode = "interval";
```

to:

```matlab
cfg.LS.derivative.K.pre_peak_mode = "peak_only";
```

### Example 5 — ES/LS optimization boundary

The ES and LS phase boundaries must remain consecutive. For example, a knee sensitivity test could change:

```matlab
cfg.phase.ES_end = 65;
cfg.phase.LS_start = 66;
```

to:

```matlab
cfg.phase.ES_end = 60;
cfg.phase.LS_start = 61;
```

`cfg.LS.scale_duration` is derived automatically from `ES_end` and `LS_peak_target`. This test changes the offline ES/LS optimization partition; it does **not** change the real-time FSM transition rule.

When testing alternative configurations, inspect the console continuity diagnostics and the generated surfaces before using the parameters in controller experiments.

## Evaluation

`evaluate_impedance_fit.m` compares reconstructed moments against the discrete AB reference trajectories.

### Trajectory metrics

The evaluator reports:

- **MAE** for full stance, ES, and LS;
- **phase-ed Pearson r²** for full stance, ES, and LS.

Each comparison interval is phase-aligned independently using normalized cross-correlation before Pearson correlation is calculated.

### Peak moment MAE

Peak-vs-peak moment fidelity is evaluated at every AB reference slope:

- level walking and ascent: positive maximum over the full stance phase;
- descent: positive maximum over samples 40–100.

The absolute peak error is summarized as MAE across terrain slopes and exported under `outputs/Evaluation/`.

## Full-domain MAT export

Running `evaluate_and_plot` creates:

```text
outputs/Impedance_model_full.mat
```

The struct `impedance_model_full` contains:

```text
knee_K_stance
knee_B_stance
knee_theta_eq_stance
knee_reconstructed_moment_stance
ankle_K_stance
ankle_B_stance
ankle_theta_eq_stance
ankle_reconstructed_moment_stance
```

Each field is exported as a **101 × 71 double matrix** in phase × slope orientation, matching the continuous biomechanics-model convention. The struct also includes:

```text
slope         % 1 × 71, -35:35 deg
stance_phase  % 101 × 1, 0:100%
```

These outputs are intended for direct comparison with phase-normalized experimental trajectories.

## Visualization

For each joint, the package generates independent figures for:

1. reconstructed moment with AB reference samples;
2. stiffness `K` with the slope-dependent arctangent upper envelope;
3. damping `B` with the slope-dependent arctangent upper envelope; and
4. equilibrium angle `theta_eq`.

The arctangent upper-envelope curves are also generated as standalone 2-D plots.

## Repository structure

```text
.
├── evaluate_and_plot.m
├── README.md
├── data/
│   ├── AB_refs.mat
│   ├── RBF_models.mat
│   └── README.md
├── outputs/                     # generated when evaluate_and_plot is run
└── src/
    ├── bounds/
    ├── config/
    ├── equilibrium/
    ├── evaluation/
    ├── io/
    ├── optimization/
    └── visualization/
```

## MATLAB requirements

- MATLAB
- Optimization Toolbox (`lsqlin`, `lsqcurvefit`)
- Signal Processing Toolbox (`xcorr`)

## Relationship to the biomechanics-modeling framework

This repository follows the continuous locomotion-biomechanics modeling package. The upstream package constructs RBF-unified knee and ankle biomechanical surfaces as functions of terrain slope and gait phase. This repository transforms those surfaces into continuous impedance parameters for prosthesis control.

## Citation

If you use this code in academic work, please cite the associated publication describing the mode-unified biomechanics and impedance-parameterization framework.
