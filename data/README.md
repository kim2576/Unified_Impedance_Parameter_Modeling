# Input data

This directory contains the biomechanics inputs used by the impedance parameterization.

- `RBF_models.mat`: `RBF_model_full`
- `AB_refs.mat`: `target_biomechanics_total`

`RBF_model_full` contains the continuous knee and ankle RBF surfaces over terrain slope and normalized gait phase. Required angular-velocity fields are:

```text
knee_angle_dot_stance
ankle_angle_dot_stance
knee_angle_dot_stance_norm
ankle_angle_dot_stance_norm
```

Each field provides `RBF_in_full_domain` in phase × slope orientation (101 × 71). `target_biomechanics_total` contains the discrete AB reference trajectories used for evaluation and plotting.

The input files are loaded by:

```matlab
inputs = load_impedance_inputs( ...
    "data/RBF_models.mat", ...
    "data/AB_refs.mat", ...
    AngularVelocityVariant="original"); % or "norm"
```

The optimizer uses the first 100 stance samples of the selected angular-velocity surface (71 × 100 after transpose).
