function bounds = build_impedance_bounds(slope, cfg)
%BUILD_IMPEDANCE_BOUNDS Construct slope-dependent K/B limits.
% Upper envelopes are arctangent fits through the configured terrain range.

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

% Fit arctangent envelopes through descent, level, and ascent anchors.

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
% Four-parameter arctangent envelope.

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

    % Reverse B_max so damping decreases with slope.
    
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
