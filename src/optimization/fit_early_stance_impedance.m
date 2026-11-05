function result = fit_early_stance_impedance(jointData, equilibrium, bounds, cfg)
%FIT_EARLY_STANCE_IMPEDANCE Fit ES K/B polynomials with constrained LSQ.

arguments
    jointData (1,1) struct
    equilibrium (1,1) struct
    bounds (1,1) struct
    cfg (1,1) struct
end

validateInputs(jointData, equilibrium, bounds, cfg);

nSlope = size(jointData.moment, 1);
ESend = cfg.phase.ES_end;
nCoeff = cfg.polynomial.n_coefficients_per_parameter;

phasePercent = jointData.phase(1:ESend);
phaseFraction = phasePercent(:) / 100;
P = build_polynomial_basis(phaseFraction, cfg.polynomial.order);

result = struct();
result.joint = cfg.joint;
result.phase = phasePercent;
result.phase_fraction = phaseFraction';
result.slope = jointData.slope;
result.K = zeros(nSlope, ESend);
result.B = zeros(nSlope, ESend);
result.moment = zeros(nSlope, ESend);
result.K_coefficients = zeros(nSlope, nCoeff);
result.B_coefficients = zeros(nSlope, nCoeff);
result.resnorm = zeros(nSlope, 1);
result.exitflag = zeros(nSlope, 1);
result.max_inequality_violation = zeros(nSlope, 1);
result.max_equality_residual = NaN(nSlope, 1);
result.solution_returned = false(nSlope, 1);

options = optimoptions('lsqlin', 'Display', 'off');
x0 = zeros(cfg.polynomial.n_unknowns, 1);

for s = 1:nSlope
    angleError = equilibrium.angle_error(s, 1:ESend)';
    angularVelocity = jointData.angular_velocity(s, 1:ESend)';
    targetMoment = jointData.moment(s, 1:ESend)';

    % Regression matrix for K/B polynomial coefficients.
    A = [P .* angleError, P .* angularVelocity];

    c = build_early_stance_constraints( ...
        phaseFraction, P, bounds.K.upper(s), bounds.B.upper(s), ...
        bounds, cfg);

    [x, resnorm, ~, exitflag] = lsqlin( ...
        A, targetMoment, ...
        c.Aineq, c.bineq, ...
        c.Aeq, c.beq, ...
        [], [], x0, options);

    % Record whether lsqlin returned coefficients.
    if isempty(x)
        result.K(s,:) = NaN;
        result.B(s,:) = NaN;
        result.moment(s,:) = NaN;
        result.K_coefficients(s,:) = NaN;
        result.B_coefficients(s,:) = NaN;
        result.resnorm(s) = NaN;
        result.exitflag(s) = exitflag;
        result.max_inequality_violation(s) = NaN;
        result.max_equality_residual(s) = NaN;
        continue
    end

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
    result.exitflag(s) = exitflag;

    if isempty(c.Aineq)
        result.max_inequality_violation(s) = 0;
    else
        result.max_inequality_violation(s) = max([0; c.Aineq*x - c.bineq]);
    end
    result.max_equality_residual(s) = max(abs(c.Aeq*x - c.beq));
end

result.target_moment = jointData.moment(:,1:ESend);
result.angle_error = equilibrium.angle_error(:,1:ESend);
result.angular_velocity = jointData.angular_velocity(:,1:ESend);
result.rmse_by_slope = sqrt(mean((result.target_moment - result.moment).^2, 2, 'omitnan'));
result.rmse_mean = mean(result.rmse_by_slope, 'omitnan');
result.rmse_std = std(result.rmse_by_slope, 'omitnan');

% Solver and feasibility diagnostics.
constraintTolerance = cfg.solver.constraint_tolerance;
result.solver_converged = result.exitflag > 0 & result.solution_returned;
result.constraint_feasible = result.solution_returned & ...
    result.max_inequality_violation <= constraintTolerance & ...
    result.max_equality_residual <= constraintTolerance;
result.strict_success = result.solver_converged & result.constraint_feasible;

% Surface availability follows returned coefficient vectors.
result.success = result.solution_returned;
result.failed_slope = result.slope(~result.solution_returned);
result.strict_failed_slope = result.slope(~result.strict_success);
result.solver_converged_but_infeasible_slope = ...
    result.slope(result.solver_converged & ~result.constraint_feasible);
result.constraint_tolerance = constraintTolerance;
result.full_range_available = all(result.solution_returned);
end

function validateInputs(jointData, equilibrium, bounds, cfg)
requiredData = {'joint','moment','angular_velocity','phase','slope'};
for i = 1:numel(requiredData)
    if ~isfield(jointData, requiredData{i})
        error('fit_early_stance_impedance:InvalidJointData', ...
            'Missing jointData.%s.', requiredData{i});
    end
end
if ~isfield(equilibrium, 'angle_error')
    error('fit_early_stance_impedance:InvalidEquilibrium', ...
        'Missing equilibrium.angle_error.');
end
if jointData.joint ~= cfg.joint
    error('fit_early_stance_impedance:JointMismatch', ...
        'jointData.joint (%s) does not match cfg.joint (%s).', ...
        jointData.joint, cfg.joint);
end
if size(jointData.moment,2) < cfg.phase.ES_end
    error('fit_early_stance_impedance:PhaseRange', ...
        'ES_end exceeds available stance-phase samples.');
end
if ~isequal(size(jointData.moment), size(jointData.angular_velocity), ...
        size(equilibrium.angle_error))
    error('fit_early_stance_impedance:SurfaceSize', ...
        'Moment, angular velocity, and angle-error surfaces must match.');
end
if numel(bounds.K.upper) ~= size(jointData.moment,1) || ...
        numel(bounds.B.upper) ~= size(jointData.moment,1)
    error('fit_early_stance_impedance:BoundsSize', ...
        'Slope-dependent bounds must match the number of terrain slopes.');
end
end
