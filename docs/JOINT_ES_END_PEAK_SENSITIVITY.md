# Joint-specific ES-end sensitivity: six Pearson r² metrics and four peak-moment cases

Run the separate MATLAB script `run_joint_ES_end_peak_sensitivity.m` from the package root. It leaves `evaluate_and_plot.m`, `run_ES_end_sensitivity.m`, input MAT files, equilibrium rules, and LSQ functions unchanged.

```matlab
validate_joint_ES_end_peak_sensitivity
run_joint_ES_end_peak_sensitivity
```

## User-adjustable options

At the top of `run_joint_ES_end_peak_sensitivity.m`:

```matlab
ES_end_values.knee  = [50 55 60 65 70];
ES_end_values.ankle = [50 55 60 65 70];
angular_velocity_variant = "original";  % or "norm"
run_fixed_window_analysis = true;
fixed_evaluation_ES_end.knee  = 65;
fixed_evaluation_ES_end.ankle = 55;
figure_visibility = "off";
save_full_fit_surfaces = false;
```

Each joint is refitted **independently**; there is no requirement that knee and ankle use the same candidate boundaries. The package's revised legacy defaults are knee=65 and ankle=55, used as **fixed evaluation windows** only unless changed above. Both have loading index 25 and late-stance stiffness reference index 75. The ankle's 15-sample equilibrium push-off progression is recomputed from each candidate's ES boundary; this is a sensitivity of the *complete configuration*, not only of the ES/LS regression split.

The values are **1-based phase sample indices** on `linspace(0,100,100)`, not exact percentage values. For example, index 65 corresponds to 64.646% stance. These are **offline identification intervals**, not commanded real-time FSM transitions.

## Six waveform reconstruction metrics, per joint

All are computed on the 17 sampled AB reference slopes by `evaluate_impedance_fit.m`, with 0° grouped into LW+Ascent, and negative slopes into Descent:

| Group | Overall | ES | LS |
|---|---|---|---|
| LW + Ascent | mean ± sample SD r² | mean ± SD r² | mean ± SD r² |
| Descent | mean ± SD r² | mean ± SD r² | mean ± SD r² |

The `phase_aligned` setting uses direct overall Pearson r² plus the package's cross-correlation-aligned ES/LS Pearson r² (as in the current Table I exporter). The `unaligned` setting is direct Pearson r² for all six cells. Both sets are exported.

## Peak-moment cases, per joint

- **Knee ascent:** peak positive (maximum) knee moment within ES for **positive** AB reference slopes.
- **Knee descent:** peak positive knee moment within LS for **negative** AB reference slopes.
- **Ankle ascent:** peak positive ankle moment within LS for positive slopes.
- **Ankle descent:** peak positive ankle moment within LS for negative slopes.

These cases **exclude the 0° condition**. Each case has 8 AB reference terrain slopes if all optimizations return values. The peak is computed with MATLAB `max` of the moment *within the designated phase interval*; it is not `max(abs(moment))`, and no circular shift or peak time realignment is performed.

At each AB reference slope, the script exports the highest moment of (1) the reconstructed LSQ model, (2) the original AB reference moment, and (3) the RBF target the optimizer attempts to reproduce. Both signed and absolute errors are recorded. Across slopes it reports mean ± SD peak magnitude, peak MAE ± SD vs AB and RBF, signed bias, and Pearson r² of **peak magnitudes across slopes**. The latter is distinct from the six waveform correlation metrics.

Two separate windows are provided:

1. `native`: each candidate's own ES and LS intervals. Boundary movement **changes which samples qualify as an ES or LS peak** and can change the AB peak itself.
2. `fixed`: every candidate for a joint uses the **same** evaluation intervals; default knee=65, ankle=55. The fits still use their actual candidate boundary. **Prefer this for comparing parameterization quality across candidate ES_end settings.**

## Files produced

Root: `outputs/Joint_ES_end_Peak_Sensitivity/<original|norm>/`

- `all_six_r2_numeric.csv`, `all_six_r2_wide.csv`, `all_six_r2_per_slope.csv`
- `all_peak_summary_numeric.csv`, `all_peak_per_slope.csv`
- `all_solver_diagnostics.csv`
- `joint_ES_end_peak_sensitivity.mat`

And each `knee/` or `ankle/` subfolder has equivalent per-joint CSVs, plus:

- `<joint>_native_phase_aligned_six_r2.png`
- `<joint>_native_unaligned_six_r2.png`
- `<joint>_fixed_phase_aligned_six_r2.png`
- `<joint>_fixed_unaligned_six_r2.png`
- `<joint>_native_peak_mae.png`, `<joint>_fixed_peak_mae.png`
- `<joint>_native_peak_magnitude.png`, `<joint>_fixed_peak_magnitude.png`
- `<joint>_native_peak_by_slope.png`, `<joint>_fixed_peak_by_slope.png`

*The `native_peak_by_slope` panel shows a candidate-matched AB peak for each ES_end, since changing the phase window can change the AB peak itself. The `fixed_peak_by_slope` panel shows one fixed-window AB reference, allowing direct candidate-to-candidate comparison.*

## Numerical caution

Changing the velocity field to `original` or `norm`, moving ES_end, and using flexion_delay=0 can change numerical reconstructions from the historical results. Solver status is tracked as `FullReturned` and `FullStrict`: a nonempty `lsqlin` output is not automatically a verified feasible optimum. Review `solver_diagnostics.csv` before selecting any preferred ES_end. The package does **not** automatically choose an optimal boundary.
