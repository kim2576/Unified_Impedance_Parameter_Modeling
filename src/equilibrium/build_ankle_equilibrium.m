function equilibrium = build_ankle_equilibrium(jointData, cfg)

%{
Reproduce the ankle equilibrium-angle rule.
This function contains only the ankle-specific equilibrium-angle policy.
Impedance bounds and constrained least-squares fitting are handled later.
%}

arguments
    jointData (1,1) struct
    cfg (1,1) struct
end

angle = jointData.angle;
moment = jointData.moment;
swingAngle = jointData.swing_angle;
slope = jointData.slope(:)';

[nSlope, nPhase] = size(angle);
if ~isequal(size(moment), [nSlope, nPhase])
    error('build_ankle_equilibrium:SurfaceSizeMismatch', ...
        'Ankle angle and moment surfaces must have the same size.');
end
if isempty(swingAngle) || ~isequal(size(swingAngle), [nSlope, nPhase])
    error('build_ankle_equilibrium:MissingSwingAngle', ...
        'The ankle equilibrium rule requires the ankle swing-angle surface.');
end
if numel(slope) ~= nSlope
    error('build_ankle_equilibrium:SlopeSizeMismatch', ...
        'jointData.slope must contain one value per surface row.');
end

ES_end = cfg.phase.ES_end;
LS_start = cfg.phase.LS_start;
ES_weight_peak = cfg.phase.ES_weight_peak;
followGain = cfg.equilibrium.follow_gain;
supportStart = cfg.equilibrium.support_start_phase;
supportEnd = cfg.equilibrium.support_search_end;
maxSupportAngle = cfg.equilibrium.max_support_angle;
pushOffDuration = cfg.equilibrium.push_off_duration;
maxTerminalAngle = cfg.equilibrium.max_terminal_angle;

validatePhaseIndices(nPhase, ES_end, LS_start, ES_weight_peak, ...
    supportStart, supportEnd, pushOffDuration);

thetaEq = zeros(nSlope, nPhase);
supportAngle = zeros(nSlope, 1);
supportAngleRaw = zeros(nSlope, 1);
minMomentIndex = zeros(nSlope, 1);
terminalAngle = zeros(nSlope, 1);

for s = 1:nSlope
    searchWindow = moment(s, supportStart:supportEnd);
    [~, minLocalIndex] = min(searchWindow);
    minIndex = minLocalIndex + supportStart - 1;
    minMomentIndex(s) = minIndex;

    supportAngleRaw(s) = angle(s, minIndex);
    supportAngle(s) = min(maxSupportAngle, supportAngleRaw(s));

    % Preserve the legacy use of the second swing-phase sample as the
    % terminal stance equilibrium target, capped at maxTerminalAngle.
    terminalAngle(s) = min(swingAngle(s, 2), maxTerminalAngle);

    thetaEq(s, :) = supportAngle(s);

    if angle(s, 1) < supportAngle(s)
        thetaEq(s, 1:ES_end) = ...
            followGain .* (angle(s,1:ES_end) - supportAngle(s)) ...
            + supportAngle(s);

        % Legacy saturation: equilibrium angle cannot exceed supportAngle.
        thetaEq(s, :) = min(thetaEq(s, :), supportAngle(s));
    else
        ES_scaled = linspace(angle(s,1), supportAngle(s), ES_weight_peak);
        thetaEq(s, 1:ES_weight_peak) = ES_scaled;
    end

    LS_scaled = linspace(supportAngle(s), terminalAngle(s), pushOffDuration);
    nHold = nPhase - pushOffDuration - ES_end;
    LS_scaled = [LS_scaled, repmat(terminalAngle(s), 1, nHold)]; %#ok<AGROW>
    thetaEq(s, LS_start:end) = LS_scaled;
end

equilibrium = struct();
equilibrium.joint = "ankle";
equilibrium.theta_eq = thetaEq;
equilibrium.support_angle = supportAngle;
equilibrium.diagnostics = struct();
equilibrium.diagnostics.support_angle_raw = supportAngleRaw;
equilibrium.diagnostics.min_moment_index = minMomentIndex;
equilibrium.diagnostics.terminal_angle = terminalAngle;
end

function validatePhaseIndices(nPhase, ES_end, LS_start, ES_weight_peak, ...
    supportStart, supportEnd, pushOffDuration)
if LS_start ~= ES_end + 1
    error('build_ankle_equilibrium:InvalidPhaseSplit', ...
        'cfg.phase.LS_start must equal cfg.phase.ES_end + 1.');
end
if ES_end < 1 || ES_end >= nPhase
    error('build_ankle_equilibrium:InvalidESEnd', ...
        'ES_end must be between 1 and nPhase-1.');
end
if ES_weight_peak < 1 || ES_weight_peak > ES_end
    error('build_ankle_equilibrium:InvalidESWeightPeak', ...
        'ES_weight_peak must lie within early stance.');
end
if supportStart < 1 || supportEnd > nPhase || supportStart > supportEnd
    error('build_ankle_equilibrium:InvalidSearchWindow', ...
        'Invalid ankle support-angle search window.');
end
if pushOffDuration < 1 || pushOffDuration > (nPhase - ES_end)
    error('build_ankle_equilibrium:InvalidPushOffDuration', ...
        'push_off_duration must fit within late stance.');
end
end
