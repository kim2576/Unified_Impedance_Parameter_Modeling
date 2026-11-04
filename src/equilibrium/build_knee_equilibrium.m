function equilibrium = build_knee_equilibrium(jointData, cfg)
%{
Construct the knee equilibrium-angle trajectory described in Section III-A.

This function contains only the knee-specific equilibrium-angle policy.
Impedance bounds and constrained least-squares fitting are handled later.
%}

arguments
    jointData (1,1) struct
    cfg (1,1) struct
end

angle = jointData.angle;
moment = jointData.moment;
slope = jointData.slope(:)';

[nSlope, nPhase] = size(angle);
if ~isequal(size(moment), [nSlope, nPhase])
    error('build_knee_equilibrium:SurfaceSizeMismatch', ...
        'Knee angle and moment surfaces must have the same size.');
end
if numel(slope) ~= nSlope
    error('build_knee_equilibrium:SlopeSizeMismatch', ...
        'jointData.slope must contain one value per surface row.');
end

ES_end = cfg.phase.ES_end;
LS_start = cfg.phase.LS_start;
ES_weight_peak = cfg.phase.ES_weight_peak;
searchStart = cfg.equilibrium.moment_search_start;
searchEnd = cfg.equilibrium.moment_search_end;
flexionDelay = cfg.equilibrium.flexion_delay;
descentCutoff = cfg.equilibrium.descent_cutoff;
minimumSupportAngle = cfg.equilibrium.minimum_support_angle;

validatePhaseIndices(nPhase, ES_end, LS_start, ES_weight_peak, ...
    searchStart, searchEnd, flexionDelay);

% Slope-dependent proportional-following factor for descending locomotion.
% Steeper descent makes theta_eq follow the measured knee angle more closely
% during weight acceptance before converging to the support angle.
cutoffIndex = find(abs(slope - descentCutoff) < 1e-12, 1, 'first');
if isempty(cutoffIndex)
    error('build_knee_equilibrium:CutoffNotInSlopeGrid', ...
        'descent_cutoff %.6g is not present in the slope grid.', descentCutoff);
end
p_mat = zeros(1, nSlope);
p_mat(1:cutoffIndex) = linspace( ...
    cfg.equilibrium.descent_follow_start, ...
    cfg.equilibrium.descent_follow_end, ...
    cutoffIndex);

thetaEq = zeros(nSlope, nPhase);
supportAngle = zeros(nSlope, 1);
supportAngleRaw = zeros(nSlope, 1);
minMomentIndex = zeros(nSlope, 1);
maxMomentIndex = zeros(nSlope, 1);

for s = 1:nSlope
    minWindow = moment(s, searchStart:searchEnd);
    maxWindow = moment(s, searchStart:end);

    [~, minLocalIndex] = min(minWindow);
    [~, maxLocalIndex] = max(maxWindow);

    % Select the joint angle associated with the mid-to-late stance moment
    % minimum; flexionDelay optionally shifts the selected support reference.
    minIndex = minLocalIndex + searchStart - 1 + flexionDelay;
    maxIndex = maxLocalIndex + searchStart - 1;

    if minIndex > nPhase
        error('build_knee_equilibrium:SupportIndexOutOfRange', ...
            ['The delayed support-angle index (%d) exceeds the available ' ...
             'phase samples (%d) at slope %.1f deg.'], ...
            minIndex, nPhase, slope(s));
    end

    minMomentIndex(s) = minIndex;
    maxMomentIndex(s) = maxIndex;

    supportAngleRaw(s) = angle(s, minIndex);
    supportAngle(s) = max(minimumSupportAngle, supportAngleRaw(s));

    if slope(s) < descentCutoff
        % During descent, use proportional yielding until theta_eq reaches the
        % support-angle reference, then hold the support reference.
        thresholdAngle = angle(s, minIndex); % raw support-reference angle before lower-angle limiting
        scaledAngle = p_mat(s) .* angle(s, :);
        useScaled = scaledAngle <= thresholdAngle;

        thetaEq(s, :) = supportAngle(s);
        thetaEq(s, useScaled) = scaledAngle(useScaled);
    else
        % Level walking and ascent use the three-region equilibrium-angle structure.
        thetaEq(s, 1:ES_end) = supportAngle(s);

        ES_scaled = linspace(angle(s,1), supportAngle(s), ES_weight_peak);
        thetaEq(s, 1:ES_weight_peak) = ES_scaled;

        nLS = nPhase - ES_end;
        LS_scaled = linspace(supportAngle(s), angle(s,end), nLS);
        thetaEq(s, LS_start:end) = LS_scaled;
    end
end

equilibrium = struct();
equilibrium.joint = "knee";
equilibrium.theta_eq = thetaEq;
equilibrium.support_angle = supportAngle;
equilibrium.diagnostics = struct();
equilibrium.diagnostics.support_angle_raw = supportAngleRaw;
equilibrium.diagnostics.min_moment_index = minMomentIndex;
equilibrium.diagnostics.max_moment_index = maxMomentIndex;
equilibrium.diagnostics.descent_follow_factor = p_mat(:);
end

function validatePhaseIndices(nPhase, ES_end, LS_start, ES_weight_peak, ...
    searchStart, searchEnd, flexionDelay)
if LS_start ~= ES_end + 1
    error('build_knee_equilibrium:InvalidPhaseSplit', ...
        'cfg.phase.LS_start must equal cfg.phase.ES_end + 1.');
end
if ES_end < 1 || ES_end >= nPhase
    error('build_knee_equilibrium:InvalidESEnd', ...
        'ES_end must be between 1 and nPhase-1.');
end
if ES_weight_peak < 1 || ES_weight_peak > ES_end
    error('build_knee_equilibrium:InvalidESWeightPeak', ...
        'ES_weight_peak must lie within early stance.');
end
if searchStart < 1 || searchEnd > nPhase || searchStart > searchEnd
    error('build_knee_equilibrium:InvalidSearchWindow', ...
        'Invalid knee moment search window.');
end
if searchEnd - searchStart + 1 + searchStart - 1 + flexionDelay > nPhase + flexionDelay
    % Kept intentionally light; the actual selected minimum is checked above.
end
end
