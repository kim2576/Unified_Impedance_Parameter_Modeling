function cfg = get_joint_config(joint)

%{
Effective knee/ankle settings from the legacy scripts.
Centralizes joint-specific numerical settings used by the modular pipeline.
Refactoring steps preserve the effective values of the legacy scripts.
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

% Numerical feasibility tolerance used to classify constrained LSQ results.
% A positive lsqlin exit flag indicates solver convergence, but the returned
% point can still carry small floating-point constraint residuals.
cfg.solver.constraint_tolerance = 1e-6;

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
% Legacy "increase to peak" condition is evaluated only at the peak phase.
cfg.LS.derivative.K.at_peak_min = -0.001;

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
        cfg.phase.ES_end = 60;
        cfg.phase.LS_start = 61;
        cfg.phase.LS_peak_target = 75;

        cfg.bounds.B.max = 0.004;
        cfg.bounds.K.lower_ratio = 0.10;
        cfg.bounds.B.lower_ratio = 0.10;

        % Effective early-stance derivative constraints from the legacy knee script.
        cfg.ES.derivative.K.pre_peak_min  =  0.010;
        cfg.ES.derivative.B.post_peak_max =  0.010;
        cfg.LS.derivative.K.post_peak_max = 0.0001;
        cfg.LS.derivative.B.post_peak_max = 0.00001;
        cfg.LS.peak_scale_mode = "descent_only";
        cfg.LS.peak_scale_cutoff_offset = 3;

        cfg.equilibrium.strategy = "knee";
        cfg.equilibrium.flexion_delay = 10;
        cfg.equilibrium.moment_search_start = 40;
        cfg.equilibrium.moment_search_end = 80;
        cfg.equilibrium.descent_follow_start = 0.99;
        cfg.equilibrium.descent_follow_end = 0.70;
        cfg.equilibrium.minimum_support_angle = 0;

    case "ankle"
        cfg.phase.ES_end = 60;
        cfg.phase.LS_start = 61;
        cfg.phase.LS_peak_target = 75;

        cfg.bounds.B.max = 0.002;
        cfg.bounds.K.lower_ratio = 0.20;
        cfg.bounds.B.lower_ratio = 0.20;

        % Effective early-stance derivative constraints from the legacy ankle script.
        cfg.ES.derivative.K.pre_peak_min  = -0.010;
        cfg.ES.derivative.B.post_peak_max =  0.005;

        cfg.LS.derivative.K.post_peak_max = 0.00001;
        cfg.LS.derivative.B.post_peak_max = 0.0;
        cfg.LS.peak_scale_mode = "all";
        cfg.LS.peak_scale_cutoff_offset = 0;

        cfg.equilibrium.strategy = "ankle";
        cfg.equilibrium.follow_gain = 1.1;
        cfg.equilibrium.support_start_phase = 40;
        cfg.equilibrium.support_search_end = 80;
        cfg.equilibrium.max_support_angle = 10;
        cfg.equilibrium.push_off_duration = 15;
        cfg.equilibrium.max_terminal_angle = 0;
end

% Peak timing is derived from the current ES/LS phase configuration.
cfg.LS.scale_duration = cfg.phase.LS_peak_target - cfg.phase.ES_end;

cfg.bounds.K.upper_min = cfg.bounds.K.upper_min_ratio * cfg.bounds.K.max;
cfg.bounds.B.upper_min = cfg.bounds.B.upper_min_ratio * cfg.bounds.B.max;
cfg.bounds.K.lower = cfg.bounds.K.lower_ratio * cfg.bounds.K.max;
cfg.bounds.B.lower = cfg.bounds.B.lower_ratio * cfg.bounds.B.max;
end
