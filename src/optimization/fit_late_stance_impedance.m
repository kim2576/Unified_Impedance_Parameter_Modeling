function result = fit_late_stance_impedance(jointData, equilibrium, bounds, early, cfg)
%{
FIT_LATE_STANCE_IMPEDANCE Fit K(phi) and B(phi) during late stance.

Uses the feasible early-stance endpoint values as LS initial boundary
conditions and preserves the current knee/ankle late-stance constrained
lsqlin formulation.

Model:
  M = K(phi) * (theta - theta_eq) + B(phi) * theta_dot
%}

arguments
    jointData (1,1) struct
    equilibrium (1,1) struct
    bounds (1,1) struct
    early (1,1) struct
    cfg (1,1) struct
end

validateInputs(jointData, equilibrium, bounds, early, cfg);

nSlope = size(jointData.moment, 1);
LSstart = cfg.phase.LS_start;
phasePercent = jointData.phase(LSstart:end);
phaseFraction = phasePercent(:) / 100;
P = build_polynomial_basis(phaseFraction, cfg.polynomial.order);
nLS = numel(phasePercent);
nCoeff = cfg.polynomial.n_coefficients_per_parameter;

result = struct();
result.joint = cfg.joint;
result.phase = phasePercent;
result.phase_fraction = phaseFraction';
result.slope = jointData.slope;
result.K = NaN(nSlope, nLS);
result.B = NaN(nSlope, nLS);
result.moment = NaN(nSlope, nLS);
result.K_coefficients = NaN(nSlope, nCoeff);
result.B_coefficients = NaN(nSlope, nCoeff);
result.resnorm = NaN(nSlope, 1);
result.exitflag = zeros(nSlope, 1);
result.max_inequality_violation = NaN(nSlope, 1);
result.max_equality_residual = NaN(nSlope, 1);
result.peak_phase_fraction = NaN(nSlope, 1);
result.peak_relative_index = NaN(nSlope, 1);
result.peak_K_target = NaN(nSlope, 1);
result.K_upper = NaN(nSlope, 1);
result.B_upper = NaN(nSlope, 1);
if isfield(early, 'solution_returned')
    earlyAvailable = early.solution_returned(:);
else
    earlyAvailable = early.success(:);
end
result.skipped_due_to_early_failure = ~earlyAvailable;
result.solution_returned = false(nSlope, 1);

options = optimoptions('lsqlin', 'Display', 'off');
x0 = zeros(cfg.polynomial.n_unknowns, 1);

for s = 1:nSlope
    % LS depends on the ES endpoint. Do not propagate an ES solution that
    % failed solver/feasibility checks into the LS fit.
    if ~earlyAvailable(s)
        continue
    end

    earlyEndK = early.K(s,end);
    earlyEndB = early.B(s,end);
    angleError = equilibrium.angle_error(s, LSstart:end)';
    angularVelocity = jointData.angular_velocity(s, LSstart:end)';
    targetMoment = jointData.moment(s, LSstart:end)';

    A = [P .* angleError, P .* angularVelocity];

    c = build_late_stance_constraints( ...
        phaseFraction, P, earlyEndK, earlyEndB, ...
        jointData.slope(s), bounds, cfg);

    [x, resnorm, ~, exitflag] = lsqlin( ...
        A, targetMoment, ...
        c.Aineq, c.bineq, ...
        c.Aeq, c.beq, ...
        [], [], x0, options);

    result.exitflag(s) = exitflag;
    result.peak_phase_fraction(s) = c.peak_phase_fraction;
    result.peak_relative_index(s) = c.peak_relative_index;
    result.peak_K_target(s) = c.peak_K_target;
    result.K_upper(s) = c.K_upper;
    result.B_upper(s) = c.B_upper;

    if isempty(x)
        continue
    end

    % Match the legacy scripts: keep every nonempty lsqlin return.
    result.solution_returned(s) = true;

    kCoeff = x(1:nCoeff);
    bCoeff = x(nCoeff+1:end);

    K = polyval(kCoeff, phaseFraction);
    B = polyval(bCoeff, phaseFraction);
    reconstructedMoment = K .* angleError + B .* angularVelocity;

    result.K(s,:) = K';
    result.B(s,:) = B';
    result.moment(s,:) = reconstructedMoment';
    result.K_coefficients(s,:) = kCoeff';
    result.B_coefficients(s,:) = bCoeff';
    result.resnorm(s) = resnorm;

    if isempty(c.Aineq)
        result.max_inequality_violation(s) = 0;
    else
        result.max_inequality_violation(s) = max([0; c.Aineq*x - c.bineq]);
    end
    result.max_equality_residual(s) = max(abs(c.Aeq*x - c.beq));
end

result.target_moment = jointData.moment(:,LSstart:end);
result.angle_error = equilibrium.angle_error(:,LSstart:end);
result.angular_velocity = jointData.angular_velocity(:,LSstart:end);
result.rmse_by_slope = sqrt(mean((result.target_moment - result.moment).^2, 2, 'omitnan'));
result.rmse_mean = mean(result.rmse_by_slope, 'omitnan');
result.rmse_std = std(result.rmse_by_slope, 'omitnan');

constraintTolerance = cfg.solver.constraint_tolerance;
result.solver_converged = result.exitflag > 0 & result.solution_returned;
result.constraint_feasible = result.solution_returned & ...
    result.max_inequality_violation <= constraintTolerance & ...
    result.max_equality_residual <= constraintTolerance;
result.strict_success = earlyAvailable & result.solver_converged & result.constraint_feasible;

% Legacy-compatible usability: a returned LS coefficient vector is retained
% even when strict convergence/feasibility diagnostics are not satisfied.
result.success = earlyAvailable & result.solution_returned;
result.failed_slope = result.slope(~result.success);
result.strict_failed_slope = result.slope(~result.strict_success);
result.solver_converged_but_infeasible_slope = ...
    result.slope(result.solver_converged & ~result.constraint_feasible);
result.constraint_tolerance = constraintTolerance;
result.full_range_available = all(result.success);
end

function validateInputs(jointData, equilibrium, bounds, early, cfg)
requiredData = {'joint','moment','angular_velocity','phase','slope'};
for i = 1:numel(requiredData)
    if ~isfield(jointData, requiredData{i})
        error('fit_late_stance_impedance:InvalidJointData', ...
            'Missing jointData.%s.', requiredData{i});
    end
end
if ~isfield(equilibrium, 'angle_error')
    error('fit_late_stance_impedance:InvalidEquilibrium', ...
        'Missing equilibrium.angle_error.');
end
requiredEarly = {'K','B','success'};
for i = 1:numel(requiredEarly)
    if ~isfield(early, requiredEarly{i})
        error('fit_late_stance_impedance:InvalidEarlyResult', ...
            'Missing early.%s.', requiredEarly{i});
    end
end
if jointData.joint ~= cfg.joint
    error('fit_late_stance_impedance:JointMismatch', ...
        'jointData.joint (%s) does not match cfg.joint (%s).', ...
        jointData.joint, cfg.joint);
end
if cfg.phase.LS_start ~= cfg.phase.ES_end + 1
    error('fit_late_stance_impedance:PhaseBoundary', ...
        'Current Step 5 expects LS_start = ES_end + 1.');
end
if size(jointData.moment,2) < cfg.phase.LS_start
    error('fit_late_stance_impedance:PhaseRange', ...
        'LS_start exceeds available stance-phase samples.');
end
if ~isequal(size(jointData.moment), size(jointData.angular_velocity), ...
        size(equilibrium.angle_error))
    error('fit_late_stance_impedance:SurfaceSize', ...
        'Moment, angular velocity, and angle-error surfaces must match.');
end
if size(early.K,1) ~= size(jointData.moment,1) || ...
        size(early.B,1) ~= size(jointData.moment,1)
    error('fit_late_stance_impedance:EarlySlopeSize', ...
        'Early-stance K/B must match the number of terrain slopes.');
end
if size(early.K,2) ~= cfg.phase.ES_end || size(early.B,2) ~= cfg.phase.ES_end
    error('fit_late_stance_impedance:EarlyPhaseSize', ...
        'Early-stance K/B phase dimensions do not match cfg.phase.ES_end.');
end
if numel(early.success) ~= size(jointData.moment,1)
    error('fit_late_stance_impedance:EarlyStatusSize', ...
        'early.success must contain one status per terrain slope.');
end
if ~isfield(bounds, 'K') || ~isfield(bounds, 'B') || ...
        ~isfield(bounds.K, 'lower') || ~isfield(bounds.B, 'lower')
    error('fit_late_stance_impedance:InvalidBounds', ...
        'Missing scalar lower bounds for K/B.');
end
end
