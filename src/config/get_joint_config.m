function cfg = get_joint_config(joint)

arguments
    joint (1,1) string {mustBeMember(joint,["knee","ankle"])}
end

cfg = struct();
cfg.joint = joint;

% Fourth-order K/B polynomials (Eq. 10).
cfg.polynomial.order = 4;
cfg.polynomial.n_coefficients_per_parameter = 5;
cfg.polynomial.n_unknowns = 10;

% Solver tolerance and impedance-envelope settings.
cfg.solver.constraint_tolerance = 1e-6;
cfg.bounds.K.max = 0.08;
cfg.bounds.K.upper_min_ratio = 0.25;
cfg.bounds.B.upper_min_ratio = 0.25;

% First vGRF peak reference (~25% stance).
cfg.phase.ES_weight_peak = 25;
% Descending-locomotion cutoff.
cfg.equilibrium.descent_cutoff = 0;

% Late-stance settings.
cfg.LS = struct();
cfg.LS.K_scale = 1.0;
cfg.LS.derivative = struct();
cfg.LS.derivative.K = struct();
cfg.LS.derivative.B = struct();
% LS1 K derivative: "interval" or "peak_only" at p_K.

% Early-stance derivative limits.
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
        % Knee phase intervals (Appendix B).
        cfg.phase.ES_end = 65;          % terminal ES sample
        cfg.phase.LS_start = 66;        % initial LS sample
        cfg.phase.LS_peak_target = 75;  % stiffness-reference phase p_K

        % Knee impedance bounds.
        cfg.bounds.B.max = 0.004;
        cfg.bounds.K.lower_ratio = 0.10;
        cfg.bounds.B.lower_ratio = 0.10;

        % Knee ES derivative limits.
        cfg.ES.derivative.K.pre_peak_min  =  0.010;
        cfg.ES.derivative.B.post_peak_max =  0.010;
        % Knee LS derivative limits.
        cfg.LS.derivative.K.pre_peak_mode = "interval"; % or "peak_only" at p_K only
        cfg.LS.derivative.K.pre_peak_min = -0.005;
        cfg.LS.derivative.K.post_peak_max = 0.0001;
        cfg.LS.derivative.B.post_peak_max = 0.00001;
        % Apply LS stiffness scaling on descent only.
        cfg.LS.peak_scale_mode = "descent_only";
        cfg.LS.peak_scale_cutoff_offset = 3;

        % Knee support-angle and descent-yielding settings.
        cfg.equilibrium.strategy = "knee";
        cfg.equilibrium.flexion_delay = 0;       % sample offset after selecting the moment minimum
        cfg.equilibrium.moment_search_start = 40; % start of support-angle search window
        cfg.equilibrium.moment_search_end = 80;   % end of support-angle search window
        cfg.equilibrium.descent_follow_start = 0.99; % steep-descent proportional-follow factor
        cfg.equilibrium.descent_follow_end = 0.70;   % factor approaching level walking
        cfg.equilibrium.minimum_support_angle = 0;   % lower bound on knee support reference

    case "ankle"
        % Ankle phase intervals (Appendix B).
        cfg.phase.ES_end = 55;          % terminal ES sample
        cfg.phase.LS_start = 56;        % initial LS sample
        cfg.phase.LS_peak_target = 75;  % stiffness-reference phase p_K

        % Ankle impedance bounds.
        cfg.bounds.B.max = 0.002;
        cfg.bounds.K.lower_ratio = 0.20;
        cfg.bounds.B.lower_ratio = 0.20;

        % Ankle ES derivative limits.
        cfg.ES.derivative.K.pre_peak_min  = -0.010;
        cfg.ES.derivative.B.post_peak_max =  0.005;

        cfg.LS.derivative.K.pre_peak_mode = "interval"; % or "peak_only" at p_K only
        cfg.LS.derivative.K.pre_peak_min = -0.005;
        cfg.LS.derivative.K.post_peak_max = 0.00001;
        cfg.LS.derivative.B.post_peak_max = 0.0;
        % Apply LS stiffness scaling at all slopes.
        cfg.LS.peak_scale_mode = "all";
        cfg.LS.peak_scale_cutoff_offset = 0;

        % Ankle support-angle, yielding, and push-off settings.
        cfg.equilibrium.strategy = "ankle";
        cfg.equilibrium.follow_gain = 1.1;              % proportional-yielding factor P
        cfg.equilibrium.support_start_phase = 40;       % start of support-angle search window
        cfg.equilibrium.support_search_end = 80;        % end of support-angle search window
        cfg.equilibrium.max_support_angle = 10;         % maximum support reference (deg dorsiflexion)
        cfg.equilibrium.push_off_duration = 15;         % R3 transition length in samples
        cfg.equilibrium.max_terminal_angle = 0;         % upper limit on terminal ankle reference (deg)
end

% ES-end to p_K interval.
cfg.LS.scale_duration = cfg.phase.LS_peak_target - cfg.phase.ES_end;

% Derived impedance bounds.
cfg.bounds.K.upper_min = cfg.bounds.K.upper_min_ratio * cfg.bounds.K.max;
cfg.bounds.B.upper_min = cfg.bounds.B.upper_min_ratio * cfg.bounds.B.max;
cfg.bounds.K.lower = cfg.bounds.K.lower_ratio * cfg.bounds.K.max;
cfg.bounds.B.lower = cfg.bounds.B.lower_ratio * cfg.bounds.B.max;
end
