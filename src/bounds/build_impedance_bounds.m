function bounds = build_impedance_bounds(slope, cfg)
%{ 
Construct slope-dependent K/B constraints.
bounds = build_impedance_bounds(slope, cfg)

This function extracts the arctangent bound construction used by the
legacy knee and ankle impedance-parameterization scripts. The numerical
formulation is intentionally preserved:
  - K upper bound increases from steep descent to steep ascent.
  - B upper bound uses the same fitted arctangent and is then reversed so
    that it decreases from steep descent to steep ascent.
  - K/B lower bounds are slope-independent constants from cfg.

Inputs
  slope : terrain-slope vector [deg]
  cfg   : joint configuration from get_joint_config()

Outputs
  bounds.K.upper       [1 x nSlope]
  bounds.K.lower       scalar
  bounds.K.upper_min   scalar
  bounds.B.upper       [1 x nSlope]
  bounds.B.lower       scalar
  bounds.B.upper_min   scalar
  bounds.fit.K.params  arctangent fit parameters
  bounds.fit.B.params  arctangent fit parameters
  bounds.fit.*.anchors fit anchor values used by the legacy formulation
%}

arguments
    slope (1,:) double {mustBeFinite, mustBeReal}
    cfg (1,1) struct
end

validateConfig(cfg);

if isempty(slope)
    error('build_impedance_bounds:EmptySlope', ...
        'Slope vector must not be empty.');
end
if any(diff(slope) <= 0)
    error('build_impedance_bounds:SlopeOrder', ...
        'Slope vector must be strictly increasing.');
end
if ~any(abs(slope) < 1e-12)
    error('build_impedance_bounds:MissingLevel', ...
        'Slope vector must include level walking (0 deg).');
end

% The legacy scripts used [-35, 0, 35]. Using the actual endpoints keeps
% the same behavior for the current domain while avoiding a hidden fixed
% range in the reusable function.

anchorSlope = [slope(1), 0, slope(end)];

[Kupper, Kfit] = fitSlopeBound( ...
    slope, anchorSlope, ...
    cfg.bounds.K.upper_min, cfg.bounds.K.max, false);

[Bupper, Bfit] = fitSlopeBound( ...
    slope, anchorSlope, ...
    cfg.bounds.B.upper_min, cfg.bounds.B.max, true);

bounds = struct();
bounds.slope = slope;

bounds.K = struct();
bounds.K.upper = Kupper;
bounds.K.lower = cfg.bounds.K.lower;
bounds.K.upper_min = cfg.bounds.K.upper_min;
bounds.K.upper_max = cfg.bounds.K.max;

bounds.B = struct();
bounds.B.upper = Bupper;
bounds.B.lower = cfg.bounds.B.lower;
bounds.B.upper_min = cfg.bounds.B.upper_min;
bounds.B.upper_max = cfg.bounds.B.max;

bounds.fit = struct();
bounds.fit.K = Kfit;
bounds.fit.B = Bfit;
end

function [upperBound, fitInfo] = fitSlopeBound( ...
    evaluationSlope, anchorSlope, upperMin, upperMax, reverseTrend)
% Preserve the legacy arctangent parameterization and initialization.

anchorValue = [ ...
    upperMin, ...
    mean([upperMin, upperMax]), ...
    upperMax];
relativeAnchorValue = anchorValue - upperMin;

model = @(p, x) p(1) .* atan(p(2) .* (x - p(3))) + p(4);
p0 = [ ...
    upperMax - upperMin, ...
    0.1, ...
    mean(anchorSlope), ...
    mean(relativeAnchorValue)];

options = optimoptions('lsqcurvefit', ...
    'Display', 'off', ...
    'Algorithm', 'levenberg-marquardt');
params = lsqcurvefit( ...
    model, p0, anchorSlope, relativeAnchorValue, [], [], options);

fitted = model(params, evaluationSlope);

if reverseTrend

    % Exact legacy damping construction:
    %   y_fit_B = -model_B(...)
    %   offset = B_upper_min - min(y_fit_B)
    %   B_upper = y_fit_B + offset
    
    fitted = -fitted;
    offset = upperMin - min(fitted);
    upperBound = fitted + offset;
else
    offset = upperMin;
    upperBound = fitted + upperMin;
end

fitInfo = struct();
fitInfo.params = params;
fitInfo.initial_params = p0;
fitInfo.anchor_slope = anchorSlope;
fitInfo.anchor_value = anchorValue;
fitInfo.relative_anchor_value = relativeAnchorValue;
fitInfo.reverse_trend = reverseTrend;
fitInfo.offset = offset;
end

function validateConfig(cfg)
requiredPaths = { ...
    {'bounds','K','max'}, ...
    {'bounds','K','upper_min'}, ...
    {'bounds','K','lower'}, ...
    {'bounds','B','max'}, ...
    {'bounds','B','upper_min'}, ...
    {'bounds','B','lower'}};

for i = 1:numel(requiredPaths)
    path = requiredPaths{i};
    value = cfg;
    for j = 1:numel(path)
        if ~isstruct(value) || ~isfield(value, path{j})
            error('build_impedance_bounds:InvalidConfig', ...
                'Missing cfg.%s.', strjoin(path, '.'));
        end
        value = value.(path{j});
    end
    if ~isscalar(value) || ~isfinite(value)
        error('build_impedance_bounds:InvalidConfig', ...
            'cfg.%s must be a finite scalar.', strjoin(path, '.'));
    end
end

if cfg.bounds.K.upper_min > cfg.bounds.K.max || ...
        cfg.bounds.B.upper_min > cfg.bounds.B.max
    error('build_impedance_bounds:InvalidConfig', ...
        'Upper-bound minima cannot exceed their maxima.');
end
if cfg.bounds.K.lower > cfg.bounds.K.upper_min || ...
        cfg.bounds.B.lower > cfg.bounds.B.upper_min
    error('build_impedance_bounds:InvalidConfig', ...
        'Lower bounds cannot exceed the minimum slope-dependent upper bound.');
end
end
