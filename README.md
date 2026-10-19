# Biomechanics-Informed Impedance Parameterization

MATLAB framework for converting continuous knee and ankle biomechanics into slope- and phase-dependent impedance parameters for lower-limb prosthesis control.

The package takes continuous able-bodied joint-angle and joint-moment surfaces as inputs and identifies stance-phase stiffness (`K`), damping (`B`), and equilibrium-angle (`theta_eq`) trajectories across terrain slopes from `-35 deg` to `+35 deg`. Knee and ankle share the same optimization pipeline while retaining joint-specific equilibrium-angle strategies and parameter settings.

## Overview

For each joint and terrain slope, the stance-phase moment is reconstructed using

```text
M = -K (theta - theta_eq) - B theta_dot
```

where `K` and `B` are represented as fourth-order polynomials of normalized stance phase. The parameterization pipeline:

1. loads the RBF-unified biomechanics and discrete able-bodied reference data;
2. selects knee or ankle kinematics and kinetics through a common interface;
3. constructs a joint-specific equilibrium-angle trajectory;
4. defines slope-dependent upper and lower bounds for stiffness and damping;
5. identifies early- and late-stance impedance parameters using constrained linear least squares;
6. combines the segment solutions into full-stance continuous surfaces;
7. evaluates reconstructed joint moments against the able-bodied reference trajectories; and
8. generates independent legacy-style figures for reconstructed moment, stiffness, damping, and equilibrium angle.

The original monolithic knee and ankle scripts are retained under `legacy/` for traceability.

## Quick start

Run the main script from the repository root:

```matlab
evaluate_and_plot
```

The script processes both the knee and ankle using the settings defined in:

```text
src/config/get_joint_config.m
```

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

These files are outputs from the preceding continuous biomechanics-modeling framework. See [`data/README.md`](data/README.md) for the expected interface.

## Parameterization pipeline

### Biomechanics interface

`select_joint_biomechanics.m` converts the joint-specific RBF fields into a common representation containing stance angle, angular velocity, moment, reference moment, terrain slope, and normalized phase. The ankle interface additionally exposes swing-angle information used in the equilibrium-angle construction.

### Equilibrium angle

The knee and ankle use separate equilibrium-angle strategies behind a common API:

```matlab
equilibrium = build_equilibrium_angle(data, cfg);
```

- **Knee:** derives the support equilibrium angle from the mid-to-late stance moment behavior, with a separate yielding strategy for descending terrain.
- **Ankle:** derives the support angle from the late-stance moment minimum, limits dorsiflexion to the configured maximum, and transitions toward the terminal ankle angle during unloading.

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

This file defines the phase segmentation, stiffness and damping bounds, derivative constraints, equilibrium-angle settings, and knee/ankle-specific parameters. Modify this file when adapting the framework to a different controller or parameterization strategy rather than editing the optimization functions directly.

## MATLAB requirements

The implementation uses functionality from:

- MATLAB;
- Optimization Toolbox (`lsqlin`, `lsqcurvefit`); and
- Signal Processing Toolbox (`xcorr`).

## Relationship to the biomechanics-modeling framework

This repository is intended to follow the continuous locomotion-biomechanics modeling pipeline. The preceding framework constructs continuous knee and ankle angle/moment surfaces as functions of terrain slope and normalized phase; this repository transforms those biomechanical surfaces into continuous impedance parameters suitable for prosthesis control.

## Citation

If you use this framework in academic work, please cite the associated publication describing the mode-unified biomechanics and impedance-parameterization method. Publication metadata can be added here once the final bibliographic record is available.
