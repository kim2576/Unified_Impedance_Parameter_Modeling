function cfg = get_joint_config(joint)

arguments
    joint (1,1) string {mustBeMember(joint,["knee","ankle"])}
end

cfg = struct();
cfg.joint = joint;

% Fourth-order stance-phase polynomial representation for K(p) and B(p)
% [Eq. (10) in the manuscript]. Five coefficients are identified for each
% parameter, giving ten LSQ unknowns per ES or LS optimization.
cfg.polynomial.order = 4;
cfg.polynomial.n_coefficients_per_parameter = 5;
cfg.polynomial.n_unknowns = 10;

% Numerical diagnostic tolerance and slope-dependent impedance-bound settings.
% K_max(s) and B_max(s) are arctangent upper envelopes; upper_min_ratio sets
% their value at the low end of each envelope relative to the configured max.
cfg.solver.constraint_tolerance = 1e-6;
% Bounds can be decoupled for knee and ankle, by separately defining in the
% switch loop below.
cfg.bounds.K.max = 0.08;
cfg.bounds.K.upper_min_ratio = 0.25;
cfg.bounds.B.upper_min_ratio = 0.25;

% Reference phase near the first vGRF peak (~25% stance). This divides the
% ES derivative-shape constraints between initial loading and single support.
cfg.phase.ES_weight_peak = 25;
% Slopes below this value use the descending-locomotion equilibrium strategy.
cfg.equilibrium.descent_cutoff = 0;

% Late-stance LSQ settings. K_scale sets the support-level stiffness target
% relative to terminal ES stiffness (1: Terminal ES = Initial LS).
% Derivative limits regulate LS1 support and LS2 unloading as described in Appendix B.
cfg.LS = struct();
cfg.LS.K_scale = 1.0;
cfg.LS.derivative = struct();
cfg.LS.derivative.K = struct();
cfg.LS.derivative.B = struct();
% LS1 stiffness derivative rule. "interval" enforces K'(p) >= pre_peak_min
% from LS start through p_K; "peak_only" applies the same bound only at p_K.
% No separate LS1 damping derivative lower bound is imposed.

% Early-stance derivative limits. These signed limits prescribe how K and B
% may progress before and after the 25% stance reference. The constraint builder
% converts them to A*x <= b inequalities.
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
        % Offline knee optimization intervals from Appendix B.
        cfg.phase.ES_end = 65;          % terminal ES sample
        cfg.phase.LS_start = 66;        % initial LS sample
        cfg.phase.LS_peak_target = 75;  % stiffness-reference phase p_K

        % Knee impedance envelopes. Lower ratios define K_min and B_min.
        cfg.bounds.B.max = 0.004;
        cfg.bounds.K.lower_ratio = 0.10;
        cfg.bounds.B.lower_ratio = 0.10;

        % Knee ES derivative limits governing loading and support progression.
        cfg.ES.derivative.K.pre_peak_min  =  0.010;
        cfg.ES.derivative.B.post_peak_max =  0.010;
        % Knee LS1 stiffness derivative constraint before the p_K reference.
        cfg.LS.derivative.K.pre_peak_mode = "interval"; % or "peak_only" at p_K only
        cfg.LS.derivative.K.pre_peak_min = -0.005;
        cfg.LS.derivative.K.post_peak_max = 0.0001;
        cfg.LS.derivative.B.post_peak_max = 0.00001;
        % Apply the LS stiffness-reference scaling condition only on the
        % descending side of the configured cutoff region.
        cfg.LS.peak_scale_mode = "descent_only";
        cfg.LS.peak_scale_cutoff_offset = 3;

        % Knee equilibrium-angle construction. The support angle is selected
        % from the mid-to-late stance moment minimum. During descent, the
        % slope-dependent follow factor controls weight-acceptance yielding.
        cfg.equilibrium.strategy = "knee";
        cfg.equilibrium.flexion_delay = 0;       % sample offset after selecting the moment minimum
        cfg.equilibrium.moment_search_start = 40; % start of support-angle search window
        cfg.equilibrium.moment_search_end = 80;   % end of support-angle search window
        cfg.equilibrium.descent_follow_start = 0.99; % steep-descent proportional-follow factor
        cfg.equilibrium.descent_follow_end = 0.70;   % factor approaching level walking
        cfg.equilibrium.minimum_support_angle = 0;   % lower bound on knee support reference

    case "ankle"
        % Offline ankle optimization intervals from Appendix B.
        cfg.phase.ES_end = 55;          % terminal ES sample
        cfg.phase.LS_start = 56;        % initial LS sample
        cfg.phase.LS_peak_target = 75;  % stiffness-reference phase p_K

        % Ankle impedance envelopes. Lower ratios define K_min and B_min.
        cfg.bounds.B.max = 0.002;
        cfg.bounds.K.lower_ratio = 0.20;
        cfg.bounds.B.lower_ratio = 0.20;

        % Ankle ES derivative limits governing loading and support progression.
        cfg.ES.derivative.K.pre_peak_min  = -0.010;
        cfg.ES.derivative.B.post_peak_max =  0.005;

        cfg.LS.derivative.K.pre_peak_mode = "interval"; % or "peak_only" at p_K only
        cfg.LS.derivative.K.pre_peak_min = -0.005;
        cfg.LS.derivative.K.post_peak_max = 0.00001;
        cfg.LS.derivative.B.post_peak_max = 0.0;
        % Apply the configured LS stiffness-reference scaling at all slopes.
        cfg.LS.peak_scale_mode = "all";
        cfg.LS.peak_scale_cutoff_offset = 0;

        % Ankle equilibrium-angle construction. The support angle is selected
        % from the mid-to-late stance moment minimum and capped at 10 deg DF.
        % follow_gain implements proportional yielding; push_off_duration sets
        % the R3 transition length toward the terminal equilibrium reference.
        cfg.equilibrium.strategy = "ankle";
        cfg.equilibrium.follow_gain = 1.1;              % proportional-yielding factor P
        cfg.equilibrium.support_start_phase = 40;       % start of support-angle search window
        cfg.equilibrium.support_search_end = 80;        % end of support-angle search window
        cfg.equilibrium.max_support_angle = 10;         % maximum support reference (deg dorsiflexion)
        cfg.equilibrium.push_off_duration = 15;         % R3 transition length in samples
        cfg.equilibrium.max_terminal_angle = 0;         % upper limit on terminal ankle reference (deg)
end

% Number of samples from terminal ES to the LS stiffness-reference phase p_K.
cfg.LS.scale_duration = cfg.phase.LS_peak_target - cfg.phase.ES_end;

% Derived envelope values used by the constrained LSQ problem.
cfg.bounds.K.upper_min = cfg.bounds.K.upper_min_ratio * cfg.bounds.K.max;
cfg.bounds.B.upper_min = cfg.bounds.B.upper_min_ratio * cfg.bounds.B.max;
cfg.bounds.K.lower = cfg.bounds.K.lower_ratio * cfg.bounds.K.max;
cfg.bounds.B.lower = cfg.bounds.B.lower_ratio * cfg.bounds.B.max;
end
