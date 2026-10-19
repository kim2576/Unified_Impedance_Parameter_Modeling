function result = assemble_full_stance_impedance(jointData, equilibrium, bounds, early, late, cfg)
%{
Combine ES and LS results into one model.

The returned surfaces follow the package convention:
  rows    = terrain slope
  columns = normalized stance phase

Legacy-compatible assembly: every nonempty ES/LS solver result is retained.
Strict convergence/constraint feasibility is stored separately as a diagnostic
and does not remove slopes from the continuous terrain surface.
%}

arguments
    jointData (1,1) struct
    equilibrium (1,1) struct
    bounds (1,1) struct
    early (1,1) struct
    late (1,1) struct
    cfg (1,1) struct
end

nSlope = numel(jointData.slope);
nPhase = numel(jointData.phase);

if size(early.K,2) + size(late.K,2) ~= nPhase
    error('assemble_full_stance_impedance:PhaseSize', ...
        'ES + LS phase samples must equal the full stance phase length.');
end

result = struct();
result.joint = cfg.joint;
result.slope = jointData.slope;
result.phase = jointData.phase;
result.K = [early.K, late.K];
result.B = [early.B, late.B];
result.moment = [early.moment, late.moment];
result.target_moment = jointData.moment;
result.equilibrium_angle = equilibrium.theta_eq;
result.angle_error = equilibrium.angle_error;
result.angular_velocity = jointData.angular_velocity;
result.bounds = bounds;
result.segments.early = early;
result.segments.late = late;
result.config = cfg;

if isfield(early,'solution_returned')
    earlyAvailable = early.solution_returned(:);
else
    earlyAvailable = early.success(:);
end
if isfield(late,'solution_returned')
    lateAvailable = late.solution_returned(:);
else
    lateAvailable = late.success(:);
end
result.success = earlyAvailable & lateAvailable;
result.failed_slope = result.slope(~result.success);
result.full_range_available = all(result.success);

if isfield(early,'strict_success') && isfield(late,'strict_success')
    result.strict_success = early.strict_success(:) & late.strict_success(:);
    result.strict_failed_slope = result.slope(~result.strict_success);
else
    result.strict_success = result.success;
    result.strict_failed_slope = result.failed_slope;
end

% Only a genuinely missing solver output is invalidated. A returned solution
% with a nonpositive exitflag or diagnostic constraint residual is preserved,
% exactly as in the legacy scripts.
missing = ~result.success;
result.K(missing,:) = NaN;
result.B(missing,:) = NaN;
result.moment(missing,:) = NaN;

result.reference.slope = jointData.reference_slope;
result.reference.phase = jointData.reference_phase;
result.reference.moment = jointData.reference_moment;

% ES -> LS continuity diagnostics. The first LS sample is constrained to the
% final ES value, so these should be near numerical precision on valid rows.
result.continuity.K = NaN(nSlope,1);
result.continuity.B = NaN(nSlope,1);
valid = result.success;
if any(valid)
    result.continuity.K(valid) = late.K(valid,1) - early.K(valid,end);
    result.continuity.B(valid) = late.B(valid,1) - early.B(valid,end);
end
end
