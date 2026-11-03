function metrics = evaluate_surface_slope_smoothness(surface, slopes, phases, amplitudeScale, slopeMask)
% EVALUATE_SURFACE_SLOPE_SMOOTHNESS  Roughness of K(s,p) or B(s,p).
%
% S: nSlope-by-nPhase; s (degrees) ascending and evenly sampled at 1 deg.
% Normalize by the joint-specific CONFIGURED UPPER ENVELOPE MAXIMUM
% (e.g. K=0.08; B=knee 0.004 or ankle 0.002). Same scale across ES_end
% candidates, so optimization output range cannot bias normalization.
% First and second differences are *along terrain slope*, NOT gait phase.
% No filtering/smoothing is applied prior to computing these metrics.
%
% slopeMask: optional logical nSlope mask. Differences ONLY use adjacent
% valid one-degree slope pairs/triples; NaN/failed slopes break sequences.
% This prevents 'strict-only' comparisons from bridging missing slopes.
arguments
    surface (:,:) double
    slopes (1,:) double
    phases (1,:) double
    amplitudeScale (1,1) double {mustBePositive}
    slopeMask (:,1) logical = true(numel(slopes),1)
end
nSlope=numel(slopes);nPhase=numel(phases);
assert(isequal(size(surface),[nSlope,nPhase]), ...
    'Surface must have nSlope rows and nPhase columns.');
assert(all(diff(slopes)>0),'Slope must be strictly increasing.');
valid=slopeMask & all(isfinite(surface),2);
dSlope=diff(slopes);
step=median(dSlope);
assert(step>0,'Invalid terrain slope spacing.');
% Only use valid adjacent samples at the nominal slope-grid resolution.
pairMask=valid(1:end-1)&valid(2:end)&abs(dSlope(:)-step)<1e-7;
tripleMask=pairMask(1:end-1)&pairMask(2:end);
D1=diff(surface,1,1)/step;  % physical slope derivative
D2=diff(surface,2,1)/(step^2);
D1(~pairMask,:)=NaN;
D2(~tripleMask,:)=NaN;
N1=abs(D1)*step/amplitudeScale;  % dimensionless per adjacent grid step
N2=abs(D2)*step^2/amplitudeScale;
metrics=struct();
metrics.NSlopeValid=nnz(valid);
metrics.NPairs=nnz(pairMask);
metrics.NTriplets=nnz(tripleMask);
metrics.RMS_FirstSlopeDifference=sqrt(mean(N1(:).^2,'omitnan'));
metrics.MeanAbs_FirstSlopeDifference=mean(N1(:),'omitnan');
metrics.P95_FirstSlopeDifference=safePercentile(N1,95);
metrics.Max_FirstSlopeDifference=safeMax(N1);
metrics.RMS_SecondSlopeDifference=sqrt(mean(N2(:).^2,'omitnan'));
metrics.MeanAbs_SecondSlopeDifference=mean(N2(:),'omitnan');
metrics.P95_SecondSlopeDifference=safePercentile(N2,95);
metrics.Max_SecondSlopeDifference=safeMax(N2);
metrics.TotalVariationNormalized=sum(abs(diff(surface,1,1)).*pairMask,1,'omitnan');
metrics.TotalVariationNormalized=mean(metrics.TotalVariationNormalized,'omitnan')/amplitudeScale;
% Zero-slope crossing: adjacent differences across (-1,0) and (0,1).
[~,idx0]=min(abs(slopes));
metrics.ZeroCrossing_MaxJump=NaN;
metrics.ZeroCrossing_RMSCurvature=NaN;
if abs(slopes(idx0))<1e-6 && idx0>1 && idx0<nSlope
    localJumps=cat(1,N1(idx0-1,:),N1(idx0,:));
    metrics.ZeroCrossing_MaxJump=safeMax(localJumps);
    if tripleMask(idx0-1)
        curv=N2(idx0-1,:);
        metrics.ZeroCrossing_RMSCurvature=sqrt(mean(curv(:).^2,'omitnan'));
    end
end
metrics.HotspotSlope=NaN;
metrics.HotspotPhase=NaN;
if isfinite(metrics.Max_SecondSlopeDifference)
    [rr,cc]=find(N2==metrics.Max_SecondSlopeDifference,1);
    if ~isempty(rr)
        metrics.HotspotSlope=slopes(rr+1); % second diff centered on slope rr+1
        metrics.HotspotPhase=phases(cc);
    end
end
metrics.CurvatureMap=NaN(size(surface));
if nSlope>2,metrics.CurvatureMap(2:end-1,:)=D2*step^2/amplitudeScale;end
end
function v=safeMax(X)
x=X(isfinite(X));
if isempty(x),v=NaN;else,v=max(x);end
end
function v=safePercentile(X,p)
x=X(isfinite(X));
if isempty(x),v=NaN;else,v=prctile(x,p);end
end
