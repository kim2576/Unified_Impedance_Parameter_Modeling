> **Historical v2 document:** Version 2.5 adds an optional interval-wide LS1 K derivative lower bound. Use `"peak_only"` to reproduce the legacy v2 constraint. See `../README_v2_5_LS1.md`.

# Joint-specific legacy audit (newest v2_2(1) files)

Source: `TRO_AB_Knee_LSQ_allslopes_stanceseparated_fromJCDat_v2_2(1).m`
and `TRO_AB_Ankle_LSQ_allslopes_stanceseparated_fromDeanDat_v2_2(1).m`.
These scripts were supplied separately; they are not reproduced in the public ZIP.
When multiple assignments exist, the **last active** MATLAB assignment wins.

## Final active configurations

| Parameter | Knee | Ankle |
|---|---:|---:|
| `ES_weight_peak` | 25 | 25 |
| `ES_end` | 65 (second active assignment) | 55 (`75` commented out) |
| `LS_start` | 66 | 56 |
| `LS_peak_target` / reference index | 75 / MATLAB index 76 | 75 / MATLAB index 76 |
| `LS_scale_duration=75-ES_end` | 10 | 20 |
| Equilibrium terminal ramp indices | 66--100 | 56--70 |
| `ankle_pushoff_duration` | N/A | 15 |
| `K_upper_const_max` | 0.08 | 0.08 |
| `B_upper_const_max` | 0.004 | 0.002 |
| `K_lower_const` | 0.008 | 0.016 |
| `B_lower_const` | 0.0004 | 0.0004 |
| `K_scale_LS` | 1 | 1 |
| `des_cutoff` | 0 | 0 |
| `follow_p` | N/A | 1.1 |
| Ankle support-angle dorsiflexion cap | N/A | 10 deg |
| Knee `flexion_delay` **in uploaded legacy** | 10 | N/A |
| Knee `flexion_delay` **in current modular code** | **0** (later user instruction) | N/A |
| Angular-velocity source in uploaded legacy | `diff(angle)` | `diff(angle)` |
| Angular-velocity source in current modular code | **preprocessed RBF field** | **preprocessed RBF field** |

**Important:** the main script is aligned with effective **parameter values**,
not guaranteed to reproduce the historical moment curves exactly. An updated
angular-velocity source and removal of knee flexion delay can change the fitted
parameter surfaces and reconstruction statistics.

## Precise sample-phase mapping

Both legacy files define `stance_phase = linspace(0,100,100)`; sample index `i`
corresponds to `(i-1)*100/99` percent of normalized stance.

- Knee last ES sample index 65: **64.65%**; first LS index 66: **65.66%**.
- Ankle last ES sample index 55: **54.55%**; first LS index 56: **55.56%**.
- Ankle support-to-terminal equilibrium interpolation ends at sample 70:
  **69.70%**. Its first value at sample 56 equals the support angle.
- Stiffness 'peak' reference index 76 is **75.76%** in both fits.
- These are **offline reference and optimization phases**, not online FSM
  transition thresholds.

## Biomechanical/methodological implications

- **Knee level walking and ascent:** equilibrium angle stays at the biological
  support-angle reference after the first 25 samples and through sample 65;
  it subsequently transitions to the terminal stance angle through sample 100.
- **Knee descent:** uses the slope-dependent proportional-angle rule; do not
  describe the above plateau/ramp as unconditional across descending slopes.
- **Ankle:** early equilibrium construction either follows RBF dorsiflexion
  with conditional saturation or interpolates from heel contact to support.
  The late equilibrium trajectory transitions to a terminal target from
  sample 56 to 70; that target is the second swing sample, capped at 0 deg.
  **The target is an equilibrium angle**, not a guarantee of mechanical push-off.
- **LS stiffness reference:** equality anchor near 75.76% in both knee and ankle
  constrained fits. This is not the ankle equilibrium transition onset.
- Ankle and knee have separate ES/LS `lsqlin` optimization problems, with
  distinct K/B envelopes and derivative constraints.

## Manuscript / Fig. 5 corrections

1. Present R1/R2/R3 as a conceptual stance organization; add a **joint-specific
   timing schematic** distinguishing ankle equilibrium transition near
   55--70% from knee terminal equilibrium transition near 65--100%.
2. In Fig. 5C, branch the pipeline by joint: `knee theta_eq` and `ankle theta_eq`
   leading to independent ES/LS constrained LSQ optimizations, then joint-specific
   K(s,p), B(s,p). Do not draw a single shared phase boundary.
3. Do not label 75% as the onset of ankle push-off. Use 'late-stance stiffness
   anchor (~76% phase)' where that numerical landmark is depicted.
4. Document exact sample indices, derivative limits, slope envelopes,
   conditional ankle yielding, and terminal ankle equilibrium target in Appendix B.
5. Keep the real-time bilateral-FSM event transition (Section IV) distinct from
   these offline phase-index choices.
