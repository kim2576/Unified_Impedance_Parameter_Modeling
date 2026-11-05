function metrics = evaluate_impedance_fit(result)
%EVALUATE_IMPEDANCE_FIT Compare reconstructed moments with AB references.
% Trajectory r^2 is computed after phase alignment within each interval.

arguments
    result (1,1) struct
end

cfg = result.config;
refSlope = result.reference.slope(:)';
refMoment = result.reference.moment;
if size(refMoment,2) < 100
    error('evaluate_impedance_fit:ReferenceSize', ...
        'Reference moment must contain at least 100 phase samples.');
end
refMoment = refMoment(:,1:100);

nRef = numel(refSlope);
modelMoment = NaN(nRef,100);
continuousIndex = NaN(1,nRef);
validReference = false(1,nRef);

for i = 1:nRef
    idx = find(abs(result.slope - refSlope(i)) < 1e-10, 1);
    if isempty(idx)
        error('evaluate_impedance_fit:ReferenceSlopeMissing', ...
            'Reference slope %.6g deg is absent from the model slope domain.', refSlope(i));
    end
    continuousIndex(i) = idx;
    if result.success(idx)
        modelMoment(i,:) = result.moment(idx,:);
        validReference(i) = true;
    end
end

ESidx = 1:cfg.phase.ES_end;
LSidx = cfg.phase.LS_start:100;

metrics = struct();
metrics.reference_slope = refSlope;
metrics.continuous_slope_index = continuousIndex;
metrics.valid_reference_slope = validReference;
metrics.reference_moment = refMoment;
metrics.model_moment = modelMoment;

metrics.r2.total = NaN(1,nRef);
metrics.r2.early = NaN(1,nRef);
metrics.r2.late = NaN(1,nRef);
metrics.mae.total = NaN(1,nRef);
metrics.mae.early = NaN(1,nRef);
metrics.mae.late = NaN(1,nRef);
metrics.phase_shift_samples.total = NaN(1,nRef);
metrics.phase_shift_samples.early = NaN(1,nRef);
metrics.phase_shift_samples.late = NaN(1,nRef);

% Peak metrics at each AB reference slope.
metrics.peak.reference = NaN(1,nRef);
metrics.peak.model = NaN(1,nRef);
metrics.peak.abs_error = NaN(1,nRef);
metrics.peak.window_start_sample = NaN(1,nRef);
metrics.peak.window_label = strings(1,nRef);

for i = 1:nRef
    if ~validReference(i)
        continue
    end

    ref = refMoment(i,:);
    pred = modelMoment(i,:);

    [metrics.r2.total(i), metrics.phase_shift_samples.total(i)] = ...
        phaseAlignedR2(ref, pred);
    [metrics.r2.early(i), metrics.phase_shift_samples.early(i)] = ...
        phaseAlignedR2(ref(ESidx), pred(ESidx));
    [metrics.r2.late(i), metrics.phase_shift_samples.late(i)] = ...
        phaseAlignedR2(ref(LSidx), pred(LSidx));

    metrics.mae.total(i) = mean(abs(ref - pred), 'omitnan');
    metrics.mae.early(i) = mean(abs(ref(ESidx) - pred(ESidx)), 'omitnan');
    metrics.mae.late(i) = mean(abs(ref(LSidx) - pred(LSidx)), 'omitnan');

    % Peak window: full stance for level/ascent; samples 40:100 for descent.
    if refSlope(i) < 0
        peakIdx = 40:100;
        metrics.peak.window_label(i) = "40-100% stance";
    else
        peakIdx = 1:100;
        metrics.peak.window_label(i) = "Full stance";
    end
    metrics.peak.window_start_sample(i) = peakIdx(1);

    refPeak = max(ref(peakIdx), [], 'omitnan');
    modelPeak = max(pred(peakIdx), [], 'omitnan');
    metrics.peak.reference(i) = refPeak;
    metrics.peak.model(i) = modelPeak;
    metrics.peak.abs_error(i) = abs(modelPeak - refPeak);
end

% Level is stored separately and included with ascent in summary metrics.
metrics.groups.descent = refSlope < 0 & validReference;
metrics.groups.level = refSlope == 0 & validReference;
metrics.groups.ascent = refSlope > 0 & validReference;
metrics.groups.lw_ascent = refSlope >= 0 & validReference;
metrics.groups.all = validReference;

metrics.summary = struct();
for groupName = ["all","descent","level","ascent","lw_ascent"]
    mask = metrics.groups.(groupName);
    metrics.summary.(groupName) = summarizeGroup(metrics, mask);
end
end

function summary = summarizeGroup(metrics, mask)
summary = struct();
summary.n_slopes = nnz(mask);
summary.r2_total = meanStd(metrics.r2.total(mask));
summary.r2_early = meanStd(metrics.r2.early(mask));
summary.r2_late = meanStd(metrics.r2.late(mask));
summary.mae_total = meanStd(metrics.mae.total(mask));
summary.mae_early = meanStd(metrics.mae.early(mask));
summary.mae_late = meanStd(metrics.mae.late(mask));
summary.peak_mae = meanStd(metrics.peak.abs_error(mask));
end

function out = meanStd(values)
out = struct('mean', mean(values, 'omitnan'), ...
             'std', std(values, 'omitnan'));
end

function r2 = squaredCorrelation(a, b)
valid = isfinite(a) & isfinite(b);
a = a(valid);
b = b(valid);
if numel(a) < 2 || std(a) == 0 || std(b) == 0
    r2 = NaN;
    return
end
C = corrcoef(a, b);
r2 = C(1,2)^2;
end

function [r2, shift] = phaseAlignedR2(ref, pred)
valid = isfinite(ref) & isfinite(pred);
if nnz(valid) < 2
    r2 = NaN;
    shift = NaN;
    return
end
ref = ref(valid);
pred = pred(valid);
[xc, lags] = xcorr(ref, pred, 'normalized');
[~, idx] = max(xc);
shift = lags(idx);
predShifted = circshift(pred, shift);
r2 = squaredCorrelation(ref, predShifted);
end
