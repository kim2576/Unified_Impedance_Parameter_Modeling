function cfg = get_joint_config(joint)

%{
Joint-specific numerical configuration based on the latest legacy scripts,
with the v2.5 interval-wide LS1 stiffness derivative option.

FINAL ACTIVE LEGACY SETTINGS (MATLAB sample indices, 100 samples per stance):
    Knee:  ES_end=65, LS_start=66, LS K reference at sample 76.
    Ankle: ES_end=55, LS_start=56, push-off equilibrium ramp=15 samples,
           terminal equilibrium reached at sample 70; LS K reference=76.

DELIBERATE DEPARTURES FROM HISTORICAL LEGACY:
    v2.5: LS1 stiffness lower derivative bound may apply to an entire interval
          instead of only the stiffness reference sample (default=interval).
    knee flexion_delay=0 (user removed prior value 10);
    preprocessed RBF angular-velocity surfaces supplied by loader, not diff.
These can change optimizer outputs; regenerate metrics and figures.
The offline ES/LS split is NOT a commanded real-time FSM transition.
%}

arguments
    joint (1,1) string {mustBeMember(joint,["knee","ankle"])}
end

cfg = struct();
cfg.joint = joint;

% Shared polynomial representation used by K(phi) and B(phi).
cfg.polynomial.order = 4;
cfg.polynomial.n_coefficients_per_parameter = 5;
cfg.polynomial.n_unknowns = 10;

% Shared slope-dependent upper-bound construction.
cfg.bounds.K.max = 0.08;
cfg.bounds.K.upper_min_ratio = 0.25;
cfg.bounds.B.upper_min_ratio = 0.25;

% Current scripts both use 25% stance as the early-stance peak location.
cfg.phase.ES_weight_peak = 25;
cfg.equilibrium.descent_cutoff = 0;

% Late-stance constrained least-squares settings shared in form across joints.
cfg.LS = struct();
cfg.LS.K_scale = 1.0;
cfg.LS.derivative = struct();
cfg.LS.derivative.K = struct();
cfg.LS.derivative.B = struct();
% v2.5 LS1: K'(p) >= pre_peak_min from LS start through p_K (inclusive).
% Set pre_peak_mode="peak_only" to reproduce the v2 single-point condition.
% The two EDITABLE JOINT-SPECIFIC values are set in the switch below.
% No damping LS1 derivative constraint is added.

% Early-stance constrained least-squares settings.
% These are expressed as direct derivative limits for readability.
% build_early_stance_constraints() converts them to A*x <= b.
cfg.ES = struct();
cfg.ES.derivative = struct();
cfg.ES.derivative.K = struct();
cfg.ES.derivative.B = struct();

cfg.ES.derivative.K.post_peak_min = -0.001;
cfg.ES.derivative.K.post_peak_max =  0.010;
cfg.ES.derivative.B.pre_peak_min  =  0.001;
cfg.ES.derivative.B.post_peak_min = -0.001;

switch joint
    case "knee"
        cfg.phase.ES_end = 65;
        cfg.phase.LS_start = 66;
        cfg.phase.LS_peak_target = 75;

        cfg.bounds.B.max = 0.004;
        cfg.bounds.K.lower_ratio = 0.10;
        cfg.bounds.B.lower_ratio = 0.10;

        % Effective early-stance derivative constraints from the legacy knee script.
        cfg.ES.derivative.K.pre_peak_min  =  0.010;
        cfg.ES.derivative.B.post_peak_max =  0.010;
        % TEST SETTINGS (KNEE): LS1 derivative rule; edit these two lines.
        cfg.LS.derivative.K.pre_peak_mode = "interval"; % "peak_only" for v2
        cfg.LS.derivative.K.pre_peak_min = -0.005;
        cfg.LS.derivative.K.post_peak_max = 0.0001;
        cfg.LS.derivative.B.post_peak_max = 0.00001;
        cfg.LS.peak_scale_mode = "descent_only";
        cfg.LS.peak_scale_cutoff_offset = 3;

        cfg.equilibrium.strategy = "knee";
        cfg.equilibrium.flexion_delay = 0;  % Explicit later user decision; legacy v2_2(1) used 10.
        cfg.equilibrium.moment_search_start = 40;
        cfg.equilibrium.moment_search_end = 80;
        cfg.equilibrium.descent_follow_start = 0.99;
        cfg.equilibrium.descent_follow_end = 0.70;
        cfg.equilibrium.minimum_support_angle = 0;

    case "ankle"
        cfg.phase.ES_end = 55;
        cfg.phase.LS_start = 56;
        cfg.phase.LS_peak_target = 75;

        cfg.bounds.B.max = 0.002;
        cfg.bounds.K.lower_ratio = 0.20;
        cfg.bounds.B.lower_ratio = 0.20;

        % Effective early-stance derivative constraints from the legacy ankle script.
        cfg.ES.derivative.K.pre_peak_min  = -0.010;
        cfg.ES.derivative.B.post_peak_max =  0.005;

        cfg.LS.derivative.K.pre_peak_mode = "interval"; % "peak_only" for v2
        cfg.LS.derivative.K.pre_peak_min = -0.005;
        cfg.LS.derivative.K.post_peak_max = 0.00001;
        cfg.LS.derivative.B.post_peak_max = 0.0;
        cfg.LS.peak_scale_mode = "all";
        cfg.LS.peak_scale_cutoff_offset = 0;

        cfg.equilibrium.strategy = "ankle";
        cfg.equilibrium.follow_gain = 1.1;
        cfg.equilibrium.support_start_phase = 40;
        cfg.equilibrium.support_search_end = 80;
        cfg.equilibrium.max_support_angle = 10;
        cfg.equilibrium.push_off_duration = 15;  % Legacy v2_2(1): sample 56..70.
        cfg.equilibrium.max_terminal_angle = 0;
end

% Peak timing is derived from the current ES/LS phase configuration.
cfg.LS.scale_duration = cfg.phase.LS_peak_target - cfg.phase.ES_end;

cfg.bounds.K.upper_min = cfg.bounds.K.upper_min_ratio * cfg.bounds.K.max;
cfg.bounds.B.upper_min = cfg.bounds.B.upper_min_ratio * cfg.bounds.B.max;
cfg.bounds.K.lower = cfg.bounds.K.lower_ratio * cfg.bounds.K.max;
cfg.bounds.B.lower = cfg.bounds.B.lower_ratio * cfg.bounds.B.max;
end
