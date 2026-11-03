# Joint-specific ES-end sensitivity: terrain-slope smoothness of K and B

## Motivation

The existing `norm.zip` sensitivity result saved **only numeric r², peak, and
solver summaries**, not the full fitted K(s,p) or B(s,p) surfaces. Therefore
roughness for the existing candidates cannot be recovered from that archive
alone. This independent script reuses fit files if previously saved with
`save_full_fit_surfaces=true`, or refits (and caches) each candidate.

Run:

```matlab
validate_joint_ES_end_surface_smoothness
run_joint_ES_end_surface_smoothness
```

User settings at the top of `run_joint_ES_end_surface_smoothness.m`:

```matlab
ES_end_values.knee  = [50 55 60 65 70];
ES_end_values.ankle = [50 55 60 65 70];
angular_velocity_variant = "norm";  % or "original"
fixed_phase_boundary.knee  = 65;
fixed_phase_boundary.ankle = 55;
reuse_existing_fits = true;
save_fit_cache = true;
plot_all_candidate_surfaces = true;
```

## Analysis

For each joint, candidate ES_end, and parameter (K and B), the script measures
cross-slope behavior on **raw**, unfiltered 71 x 100 surfaces:

- Primary: root-mean-square second finite difference along slope (1° grid),
  normalized by the **fixed configuration maximum** (knee/ankle K = 0.08;
  knee B=0.004; ankle B=0.002). Lower means less oscillatory cross-slope
  variation. The normalization is **not** candidate-specific.
- RMS and mean absolute first slope differences, 95th percentile and maximum
  neighbor-to-neighbor jumps, and phase-averaged total variation in slope.
- Curvature at 0° where ascending and descending models meet, and the slope/
  phase location of the worst curvature.
- Three time windows: full stance, fixed knee 65/ankle 55 ES, and fixed LS.
  The fit boundary moves with each candidate but the *evaluation windows do not*.
- All returned solutions (legacy behavior); independently the set of slopes
  that are strictly feasible for **all candidates** of that joint. The latter
  comparison uses only adjacent one-degree triples and does not bridge gaps.

Outputs in `outputs/Joint_ES_end_Surface_Smoothness/<norm|original>/`:

- `all_surface_smoothness_summary.csv`, `all_solver_status.csv`.
- `<joint>/surface_smoothness_summary.csv`, `solver_status.csv`.
- `<joint>/<joint>_surface_smoothness_comparison.png`: roughness, worst jump,
  0° curvature, total variation (K and B curves vs ES_end).
- `<joint>/<joint>_K_fixed_phase_slices.png` and `_B_...`:
  slope profiles at 25%, 55%, 75%, and 95% stance.
- `<joint>/<joint>_ES##_K_surface_curvature.png` and corresponding B figures:
  3D parameter surface + 2D absolute curvature map with hotspot.
- `<joint>/fits/fit_ES_end_##.mat`: optional cached complete optimizer result.

## Interpreting the tradeoff

**Do not optimize smoothness alone**: a flat, unphysiological impedance surface
would score perfectly. Interpret roughness alongside the four peak-moment
MAEs and the six moment waveform r² metrics from the existing sensitivity
analysis. Also consider strict solver feasibility, value discontinuity at the
ES/LS boundary, and slope-wise zero-crossing artifacts. The smoothness index
is descriptive, **not** evidence of a unique mathematical optimum or a
validated physical continuity criterion.

The sweep changes both the equilibrium-angle trajectory and offline
optimization boundary. The candidate split indices are **not** the real-time
FSM transition timings.

No MATLAB Optimization Toolbox execution was performed in generating this
package; use MATLAB to compute the actual roughness values.
