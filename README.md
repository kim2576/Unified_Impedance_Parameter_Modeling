# Biomechanics-Informed Impedance Parameterization

> **v2.5 update:** The late-stance LS1 stiffness derivative lower bound can now be enforced over the entire LS-start-to-sample-76 interval, independently for knee and ankle. It can be reverted to the original single-point rule. See [README_v2_5_LS1.md](README_v2_5_LS1.md). Any prepackaged `outputs/` files are previous v2 results and must be regenerated for v2.5.

MATLAB framework for converting continuous knee and ankle biomechanics into slope- and phase-dependent impedance parameters for lower-limb prosthesis control.

The package takes continuous able-bodied joint-angle, **preprocessed angular-velocity**, and joint-moment surfaces as inputs and identifies stance-phase stiffness (`K`), damping (`B`), and equilibrium-angle (`theta_eq`) trajectories across terrain slopes from `-35 deg` to `+35 deg`. Knee and ankle share the same optimization pipeline while retaining joint-specific equilibrium-angle strategies and parameter settings.

## Overview

For each joint and terrain slope, the stance-phase moment is reconstructed using

```text
M = K (theta - theta_eq) + B theta_dot
```

where `K` and `B` are represented as fourth-order polynomials of normalized stance phase. The parameterization pipeline:

1. loads the RBF-unified biomechanics (including independently modeled angular velocities) and discrete able-bodied reference data;
2. selects knee or ankle kinematics and kinetics through a common interface;
3. constructs a joint-specific equilibrium-angle trajectory;
4. defines slope-dependent upper and lower bounds for stiffness and damping;
5. identifies early- and late-stance impedance parameters using constrained linear least squares;
6. combines the segment solutions into full-stance continuous surfaces;
7. evaluates reconstructed joint moments against the able-bodied reference trajectories;
8. exports six-metric reconstruction summary tables and numeric CSVs; and
9. generates independent legacy-style figures for reconstructed moment, stiffness, damping, and equilibrium angle.

Historical plots bundled under `outputs/Plots/` were generated prior to the angular-velocity change; rerun `evaluate_and_plot` to regenerate them. No original monolithic `legacy/` scripts are included in this ZIP.

## Quick start

Run the main script from the repository root:

```matlab
evaluate_and_plot
```

At the top of `evaluate_and_plot.m`, choose the preprocessed angular-velocity variant:

```matlab
angular_velocity_variant = "original"; % or "norm"
```

- **`original` (default):** `knee_angle_dot_stance` and `ankle_angle_dot_stance` RBF surfaces, fitted upstream from the recorded/preprocessed angular-velocity trajectories.
- **`norm`:** `knee_angle_dot_stance_norm` and `ankle_angle_dot_stance_norm` RBF surfaces, fitted upstream from velocities obtained by finite-differencing the averaged angle trajectories under the previously specified phase-to-time assumption.

Both modes are read **directly** from `RBF_model_full` in `data/RBF_models.mat`; neither mode differentiates joint-angle surfaces inside this package. Select the same mode for knee and ankle for comparable model evaluations.

The script processes both the knee and ankle **independently**, with joint-specific timing and optimization constraints defined in:

```text
src/config/get_joint_config.m
```

The published-code default now uses knee ES_end=65 (LS_start=66) and ankle
ES_end=55 (LS_start=56); these are 1-based sample indices on a 100-sample
`linspace(0,100,100)` phase grid, **not exact percentages**. The ankle
support-to-terminal equilibrium transition uses 15 samples (indices 56--70),
whereas the knee level/ascent terminal transition spans indices 66--100.
Both joint optimizations anchor LS stiffness at sample index 76, independent
of equilibrium-angle push-off onset. This is an offline design choice and
is **not** the online event-based FSM transition.

The user subsequently set `flexion_delay=0`; the historical knee v2_2(1)
script still contains 10. In addition, the updated workflow uses RBF-modeled,
preprocessed angular velocities instead of the historical `diff(angle)`
calculation. Hence the numerical settings reproduce the latest legacy scripts
apart from these **two declared methodological changes**. Re-run and verify
moment reconstruction before using previous figures or Table I statistics.
See `docs/LEGACY_V2_2_1_ALIGNMENT.md` for all implementation details.

It reports moment-reconstruction metrics in the Command Window and generates the visualization outputs in:

```text
outputs/Plots/
```

## Inputs

The package expects two MATLAB files under `data/`:

```text
data/
├── RBF_models.mat
└── AB_refs.mat
```

`RBF_models.mat` must contain:

```matlab
RBF_model_full
```

and `AB_refs.mat` must contain:

```matlab
target_biomechanics_total
```

The RBF model must also include the following fields, each containing
`RBF_in_full_domain` with dimensions **101 × 71** (phase × terrain slope):

```text
knee_angle_dot_stance
ankle_angle_dot_stance
knee_angle_dot_stance_norm
ankle_angle_dot_stance_norm
```

The selected velocity surface is cropped to the first 100 phase samples and transposed to **71 × 100**, matching the angle and moment inputs. The original and `_norm` variants should represent angular velocity in **deg/s**, using the upstream dataset convention. They are not converted again in this package.

These files are outputs from the preceding continuous biomechanics-modeling framework. See [`data/README.md`](data/README.md) for the expected interface.

## Parameterization pipeline

### Biomechanics interface

`select_joint_biomechanics.m` converts the joint-specific RBF fields into a common representation containing stance angle, **selected preprocessed angular velocity (rather than the finite difference of the modeled angle)**, moment, reference moment, terrain slope, and normalized phase. The ankle interface additionally exposes swing-angle information used in the equilibrium-angle construction.

To check the available velocity data and confirm its exact mapping into the 71 × 100 input arrays, run:

```matlab
validate_angular_velocity_inputs
```

The supplied `RBF_models.mat` already contains all four required velocity surfaces. Because damping is multiplied by angular velocity, replacing the velocity source will change the fitted coefficients and reconstructed moments; rerun the optimization before using previous figures or quantitative metrics.

### Equilibrium angle

The knee and ankle use separate equilibrium-angle strategies behind a common API:

```matlab
equilibrium = build_equilibrium_angle(data, cfg);
```

- **Knee:** derives the support equilibrium angle from the mid-to-late stance moment behavior, with a separate yielding strategy for descending terrain. For level/ascent, the equilibrium reference is held until ES sample 65 and transitions toward the terminal stance angle over samples 66--100.
- **Ankle:** derives the support angle from the mid-to-late stance moment minimum, caps the support reference at 10 deg dorsiflexion, and initiates its support-to-terminal plantarflexion reference at sample 56, reaching the terminal target by sample 70. A conditional proportional-following rule is used during earlier stance.

### Slope-dependent impedance bounds

`build_impedance_bounds.m` generates continuous stiffness and damping envelopes across terrain slope. The upper bounds are represented using the same arctangent formulation used in the legacy implementation, while constant lower bounds are defined from joint-specific ratios.

### Constrained optimization

Early- and late-stance stiffness and damping are identified using `lsqlin`. For each segment,

```matlab
K(phi) = k4*phi^4 + k3*phi^3 + k2*phi^2 + k1*phi + k0
B(phi) = b4*phi^4 + b3*phi^3 + b2*phi^2 + b1*phi + b0
```

which yields 10 unknown coefficients per optimization. Equality, bound, and derivative-shape constraints are assembled separately for early and late stance.

To reproduce the behavior of the original scripts across the complete slope domain, every nonempty coefficient vector returned by `lsqlin` is retained. Solver-status and constraint-residual information remain available internally for diagnostic purposes but do not mask returned full-range solutions.

## Evaluation

`evaluate_impedance_fit.m` compares the reconstructed moment against the discrete able-bodied reference trajectories at the available reference slopes.

Reported metrics include:

- squared Pearson correlation coefficient (`R^2`);
- phase-aligned early-stance and late-stance `R^2` using cross-correlation;
- root-mean-square error (RMSE); and
- mean absolute error (MAE).

Metrics are available for the full stance phase as well as the early- and late-stance regions. Summary values are also separated into descent, level, ascent, and all-terrain groups.

## Paper-style six-column reconstruction R^2 table

After each knee/ankle fit, `evaluate_and_plot.m` automatically aggregates the
reference-slope evaluations and exports a figure and numerical results under
`outputs/Evaluation/`. **No extra optimization pass is required.**

For each joint, six mean ± 1 sample SD values are calculated:

| Terrain group | Overall stance | Early stance (ES) | Late stance (LS) |
| --- | --- | --- | --- |
| LW + Ascent | 0° and positive AB-reference slopes | Same slopes | Same slopes |
| Descent | Negative AB-reference slopes | Same slopes | Same slopes |

Each slope supplies one squared Pearson correlation coefficient against its
discrete AB moment-reference trajectory. The summary takes the **arithmetic
mean and sample SD across slopes**, omitting unavailable/nonfinite metrics.
Level walking appears *only* in the LW + Ascent group (normally nine slopes,
versus eight descent slopes). ES and LS use the knee/ankle phase split from
`get_joint_config.m` (currently 60 and 40 samples).

The original evaluator contains both direct squared correlation and the
legacy cross-correlation-based **phase-aligned** ES/LS correlation. To make
this scientifically unambiguous, the new outputs distinguish:

- `phase_aligned`: overall = direct `r^2`; ES and LS = phase-aligned `r^2`;
- `unaligned`: overall, ES and LS = direct `r^2` (no shift).

Files generated in `outputs/Evaluation/`:

```text
reconstruction_r2_summary_phase_aligned.png  # image matching Table I layout
reconstruction_r2_summary_phase_aligned.csv  # 2-row x 6-summary table
reconstruction_r2_summary_unaligned.png
reconstruction_r2_summary_unaligned.csv
reconstruction_r2_numeric_phase_aligned.csv  # 12 rows with mean, SD, n
reconstruction_r2_numeric_unaligned.csv
reconstruction_r2_per_slope.csv             # all reference slopes, both metrics
reconstruction_r2_summary.mat               # full summary & source metrics
```

These files are calculated from the **current** fitted model and selected
angular-velocity variant. Figures bundled from a previous run must not be
interpreted as current results until `evaluate_and_plot` completes in MATLAB.
To test the summary logic without Optimization Toolbox or an impedance fit:

```matlab
validate_reconstruction_r2_summary
```

The sample table in the manuscript should not be assumed to match results
using the newly selected preprocessed angular-velocity inputs until the fit
has been recomputed. If the table reports phase-aligned ES/LS values,
explicitly name that metric in its caption; phase-alignment here uses the
package's legacy cross-correlation/circular-shift convention.

## Visualization

`plot_impedance_results.m` produces four independent figures for each joint:

1. reconstructed joint moment with the able-bodied reference shown as red points;
2. stiffness `K`;
3. damping `B`; and
4. equilibrium angle `theta_eq`.

The stiffness and damping plots include the slope-dependent arctangent upper-bound curves as thick green dashed lines, matching the legacy visualization convention. Figure dimensions and camera views are also retained from the original knee and ankle scripts.

Generated filenames are:

```text
knee_reconstructed_moment.png
knee_stiffness_K.png
knee_damping_B.png
knee_equilibrium_angle.png
ankle_reconstructed_moment.png
ankle_stiffness_K.png
ankle_damping_B.png
ankle_equilibrium_angle.png
```

## Repository structure

```text
.
├── evaluate_and_plot.m
├── validate_angular_velocity_inputs.m
├── README.md
├── data/
│   ├── AB_refs.mat
│   ├── RBF_models.mat
│   └── README.md
├── outputs/
└── src/
    ├── bounds/
    │   └── build_impedance_bounds.m
    ├── config/
    │   └── get_joint_config.m
    ├── equilibrium/
    │   ├── build_equilibrium_angle.m
    │   ├── build_knee_equilibrium.m
    │   └── build_ankle_equilibrium.m
    ├── evaluation/
    │   ├── assemble_full_stance_impedance.m
    │   └── evaluate_impedance_fit.m
    ├── io/
    │   ├── load_impedance_inputs.m
    │   └── select_joint_biomechanics.m
    ├── optimization/
    │   ├── build_polynomial_basis.m
    │   ├── build_early_stance_constraints.m
    │   ├── build_late_stance_constraints.m
    │   ├── fit_early_stance_impedance.m
    │   └── fit_late_stance_impedance.m
    └── visualization/
        └── plot_impedance_results.m
```

## Configuration

Joint-specific values are centralized in:

```text
src/config/get_joint_config.m
```

This file defines the **separate knee/ankle phase segmentation**, stiffness and damping bounds, derivative constraints, equilibrium-angle settings, and knee/ankle-specific parameters. Modify this file when adapting the framework to a different controller or parameterization strategy rather than editing the optimization functions directly.

Run `validate_legacy55_alignment.m` to verify these defaults and check the resulting equilibrium-angle surfaces without solving the LSQ problems.

## MATLAB requirements

The implementation uses functionality from:

- MATLAB;
- Optimization Toolbox (`lsqlin`, `lsqcurvefit`); and
- Signal Processing Toolbox (`xcorr`).

## Relationship to the biomechanics-modeling framework

This repository is intended to follow the continuous locomotion-biomechanics modeling pipeline. The preceding framework constructs continuous knee and ankle angle/moment surfaces as functions of terrain slope and normalized phase; this repository transforms those biomechanical surfaces into continuous impedance parameters suitable for prosthesis control.

## Citation

If you use this framework in academic work, please cite the associated publication describing the mode-unified biomechanics and impedance-parameterization method. Publication metadata can be added here once the final bibliographic record is available.

## ES-end sensitivity analysis (50%, 55%, 60%, 65%, 70% candidates)

Run `run_ES_end_sensitivity.m` from the project root to compare the six
Table I-style reconstruction metrics across alternative **offline**
ES-to-LS optimization boundaries. This does **not** alter the standard
`evaluate_and_plot.m` run or the online, bilateral-sensing FSM transition.

At the beginning of `run_ES_end_sensitivity.m`, edit:

```matlab
ES_end_values = [50, 55, 60, 65, 70];
angular_velocity_variant = "original"; % or "norm"
fixed_evaluation_ES_end = 60;
run_fixed_window_analysis = true;
figure_visibility = "off";
save_full_fit_surfaces = false;
```

First run the inexpensive preflight test (uses the full MAT inputs and
constructs the equality/inequality constraints but does **not** run `lsqlin`):

```matlab
validate_ES_end_sensitivity
run_ES_end_sensitivity
```

The sweep updates **all dependent configuration fields** for both knee and
ankle: `cfg.phase.ES_end`, `cfg.phase.LS_start = ES_end+1`, and
`cfg.LS.scale_duration = cfg.phase.LS_peak_target-ES_end`. Each candidate
reruns the equilibrium-angle construction and sequential ES/LS `lsqlin`
fits across all 71 terrain slopes before comparing AB moment reconstruction
at the 17 reference slopes. The 75%-reference stiffness constraint is retained.
The optimizer's 100-sample phase grid is `linspace(0,100,100)`; an ES_end
value of 60 means **60 samples**, with the last ES point near 59.60% and the
first LS point near 60.61%, not an exact 60.00% timestamp.

### Metrics and output directory

Six metrics are computed for **each joint** and candidate: LW + ascent
(Overall, ES, LS) and descent (Overall, ES, LS), as arithmetic mean and
sample SD of the per-slope squared Pearson correlation coefficients.
LW + ascent includes 0 degrees and positive reference slopes; descent
contains negative reference slopes. Both alignment conventions are exported:

- **Phase-aligned:** overall correlation is unaligned; regional ES/LS
  metrics use the existing circular-shift cross-correlation alignment.
- **Unaligned:** overall and regional correlations are calculated without
  temporal shifting.

Two phase-window conventions are also exported:

- **Native:** ES and LS metrics cover each candidate's own optimization
  intervals. These intervals differ between candidate runs, so regional
  metric values are not perfectly like-for-like.
- **Fixed (recommended for boundary comparison):** ES and LS metrics use a
  common evaluation boundary `fixed_evaluation_ES_end` (60 by default),
  while the underlying fitted moment trajectories remain unchanged. This
  controls the evaluation-window difference but not the fact that changing
  `ES_end` also modifies the knee/ankle equilibrium-angle trajectories.

Saved to `outputs/ES_end_Sensitivity/<original|norm>/`:

| File | Contents |
| --- | --- |
| `native_phase_aligned.png`, `native_unaligned.png` | Six-panel knee-versus-ankle mean +/- SD plots |
| `fixed_phase_aligned.png`, `fixed_unaligned.png` | Identical-window comparison figures |
| `*_numeric.csv` | Numeric long-form mean, sample SD, and number of slopes for six metrics |
| `*_wide.csv` | Paper-style six-column `mean +/- SD` strings for each ES_end and joint |
| `*_per_slope.csv` | Individual terrain-slope correlations for audit/paired analysis |
| `solver_diagnostics.csv` | Per-candidate counts of returned solutions, strict successes, exit flags, and ES/LS continuity errors |
| `solver_by_slope.csv` | Exit flags, inequality/equality residuals, continuity and feasibility for every slope and candidate |
| `ES_end_sensitivity_results.mat` | Complete configurations, all metrics, and summaries |

Optional `save_full_fit_surfaces=true` also saves a full `result` structure
for each joint/boundary (including the reconstructed moments and K/B surfaces).

**Interpretation caution.** A nonempty `lsqlin` solution is used by the
existing pipeline even when solver exit flags or post-hoc constraint
residuals do not meet strict feasibility criteria. Compare
`solver_diagnostics.csv` before ranking boundaries. Also, Pearson r^2
measures waveform correlation rather than torque-amplitude agreement; the
study should not claim that the boundary with the highest r^2 necessarily
has the lowest moment reconstruction MAE or the best physical behavior.

The original `get_joint_config.m` defaults (ES_end=60), the normal
`evaluate_and_plot.m` workflow, and the original MAT inputs are unchanged.


## Scope of the ES_end sensitivity study

`run_ES_end_sensitivity.m` intentionally replaces **both** joint-specific
baseline ES boundaries with each candidate in `[50,55,60,65,70]`. The
sensitivity study is therefore **not** the same configuration as the standard
`evaluate_and_plot.m` run (knee 65 / ankle 55). The 75-index stiffness
reference and the 15-sample ankle equilibrium transition remain separately
configured. Results should be labeled as joint-specific boundary sensitivity,
not exact reproduction of the manuscript's historical impedance parameters.

### Joint-specific ES boundary and peak-moment sensitivity

The separate script `run_joint_ES_end_peak_sensitivity.m` sweeps knee and ankle
`cfg.phase.ES_end` **independently** and exports six stance-waveform Pearson
r² statistics plus the knee-ascent ES, knee-descent LS, and ankle-ascent/descent
LS peak-moment errors. Native and fixed phase-evaluation windows are supported.
See `docs/JOINT_ES_END_PEAK_SENSITIVITY.md` for the exact metrics, custom lists,
plots, numerical exports and numerical caveats. Quick preflight:

```matlab
validate_joint_ES_end_peak_sensitivity
run_joint_ES_end_peak_sensitivity
```
