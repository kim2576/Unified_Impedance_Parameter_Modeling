function [perSlope, summary] = evaluate_joint_ES_end_peaks(result, esEndForEvaluation, fitESEnd, windowType)
%EVALUATE_JOINT_ES_END_PEAKS Peak moment fidelity for selected joint/terrain/phase cases.
%
% Reference definitions (same 17 AB reference slopes as evaluate_impedance_fit):
%  knee  : ascent (s>0), ES; descent (s<0), LS
%  ankle : ascent (s>0), LS; descent (s<0), LS
%
% For every AB reference slope, compute the POSITIVE MAXIMUM of the actual
% reconstructed moment, the AB measured reference, and the modeled RBF
% target over IDENTICAL indices. They are evaluated separately in native
% (candidate boundary) and fixed (joint baseline boundary) windows.
%
% Outputs include per-slope peak values, signed/absolute error vs AB and
% RBF, reference/model peak timing, and across-slope mean +/- sample SD,
% MAE +/- SD, bias, and squared Pearson r for peak magnitudes.
%
% This is a peak MAGNITUDE comparison, not a peak-timing-aligned r2 of a
% whole waveform; no phase shifting is applied.

arguments
    result (1,1) struct
    esEndForEvaluation (1,1) double {mustBeInteger,mustBePositive}
    fitESEnd (1,1) double {mustBeInteger,mustBePositive}
    windowType (1,1) string
end

nPhase = size(result.moment,2);
if esEndForEvaluation >= nPhase
    error('evaluate_joint_ES_end_peaks:Window', 'Evaluation ES end must be before last phase.');
end

joint = string(result.joint);
if joint == "knee"
    groupNames = ["Ascent","Descent"];
    phaseNames = ["ES","LS"];
elseif joint == "ankle"
    groupNames = ["Ascent","Descent"];
    phaseNames = ["LS","LS"];
else
    error('evaluate_joint_ES_end_peaks:Joint', 'Expected knee or ankle.');
end

slope = result.reference.slope(:);
refMatrix = result.reference.moment;
if size(refMatrix,1) ~= numel(slope)
    error('evaluate_joint_ES_end_peaks:ReferenceSize', 'Reference rows must correspond to AB slopes.');
end
refMatrix = refMatrix(:,1:nPhase);

vFitEnd = []; vEvaluationEnd = []; vWindow = strings(0,1);
vJoint = strings(0,1); vGroup = strings(0,1); vPhase = strings(0,1);
vSlope = []; vModel = []; vAB = []; vRBF = [];
vErrorAB = []; vAbsAB = []; vErrorRBF = []; vAbsRBF = [];
vTimeModel = []; vTimeAB = []; vTimeRBF = [];
vValidFit = [];

for caseId = 1:numel(groupNames)
    group = groupNames(caseId);
    region = phaseNames(caseId);
    if region == "ES"
        ids = 1:esEndForEvaluation;
    else
        ids = (esEndForEvaluation+1):nPhase;
    end
    if group == "Ascent"
        included = find(slope > 0);
    else
        included = find(slope < 0);
    end

    for k = included(:)'
        s = slope(k);
        modelIndex = find(abs(result.slope(:)-s)<1e-10,1);
        if isempty(modelIndex)
            error('evaluate_joint_ES_end_peaks:SlopeMissing','Slope %.2f absent from fitted domain.',s);
        end
        isValid = result.success(modelIndex);
        modelPk = NaN; modelTm = NaN; rbfPk = NaN; rbfTm = NaN;
        ref = refMatrix(k,ids);
        [abPk, abLocal] = max(ref);
        abTm = result.phase(ids(abLocal));
        target = result.target_moment(modelIndex,ids);
        if all(isfinite(target))
            [rbfPk,rbfLocal] = max(target);
            rbfTm = result.phase(ids(rbfLocal));
        end
        if isValid
            modelVals = result.moment(modelIndex,ids);
            if all(isfinite(modelVals))
                [modelPk, modelLocal] = max(modelVals);
                modelTm = result.phase(ids(modelLocal));
            else
                isValid = false;
            end
        end
        vFitEnd(end+1,1) = fitESEnd; %#ok<AGROW>
        vEvaluationEnd(end+1,1) = esEndForEvaluation; %#ok<AGROW>
        vWindow(end+1,1) = windowType; %#ok<AGROW>
        vJoint(end+1,1) = joint; %#ok<AGROW>
        vGroup(end+1,1) = group; %#ok<AGROW>
        vPhase(end+1,1) = region; %#ok<AGROW>
        vSlope(end+1,1) = s; %#ok<AGROW>
        vModel(end+1,1) = modelPk; %#ok<AGROW>
        vAB(end+1,1) = abPk; %#ok<AGROW>
        vRBF(end+1,1) = rbfPk; %#ok<AGROW>
        vErrorAB(end+1,1) = modelPk-abPk; %#ok<AGROW>
        vAbsAB(end+1,1) = abs(modelPk-abPk); %#ok<AGROW>
        vErrorRBF(end+1,1) = modelPk-rbfPk; %#ok<AGROW>
        vAbsRBF(end+1,1) = abs(modelPk-rbfPk); %#ok<AGROW>
        vTimeModel(end+1,1) = modelTm; %#ok<AGROW>
        vTimeAB(end+1,1) = abTm; %#ok<AGROW>
        vTimeRBF(end+1,1) = rbfTm; %#ok<AGROW>
        vValidFit(end+1,1) = isValid; %#ok<AGROW>
    end
end

perSlope = table(vFitEnd,vEvaluationEnd,vWindow,vJoint,vGroup,vPhase, ...
    vSlope,vValidFit,vModel,vAB,vRBF,vErrorAB,vAbsAB,vErrorRBF,vAbsRBF, ...
    vTimeModel,vTimeAB,vTimeRBF, ...
    'VariableNames',{'ES_end','EvaluationES_end','EvaluationWindow','Joint', ...
    'TerrainGroup','PhaseRegion','SlopeDeg','ValidFit','ModelPeakNmKg', ...
    'ABPeakNmKg','RBFPeakNmKg','SignedErrorAB','AbsErrorAB', ...
    'SignedErrorRBF','AbsErrorRBF','ModelPeakPhasePct','ABPeakPhasePct', ...
    'RBFPeakPhasePct'});

summary = table();
for k = 1:numel(groupNames)
    subset = perSlope(perSlope.TerrainGroup==groupNames(k),:);
    valid = subset.ValidFit & isfinite(subset.ModelPeakNmKg) ...
        & isfinite(subset.ABPeakNmKg) & isfinite(subset.RBFPeakNmKg);
    subset = subset(valid,:);
    n = height(subset);
    if n > 0
        [modelMean,modelSD] = meanStd(subset.ModelPeakNmKg);
        [abMean,abSD] = meanStd(subset.ABPeakNmKg);
        [rbfMean,rbfSD] = meanStd(subset.RBFPeakNmKg);
        [maeAB,sdAbsAB] = meanStd(subset.AbsErrorAB);
        [maeRBF,sdAbsRBF] = meanStd(subset.AbsErrorRBF);
        signedBiasAB = mean(subset.SignedErrorAB);
        signedBiasRBF = mean(subset.SignedErrorRBF);
        acrossSlopeR2_AB = squaredPearson(subset.ABPeakNmKg,subset.ModelPeakNmKg);
        acrossSlopeR2_RBF = squaredPearson(subset.RBFPeakNmKg,subset.ModelPeakNmKg);
        timeMAE_AB = mean(abs(subset.ModelPeakPhasePct - subset.ABPeakPhasePct));
    else
        [modelMean,modelSD,abMean,abSD,rbfMean,rbfSD,maeAB,sdAbsAB, ...
            maeRBF,sdAbsRBF,signedBiasAB,signedBiasRBF,acrossSlopeR2_AB, ...
            acrossSlopeR2_RBF,timeMAE_AB] = deal(NaN);
    end
    next = table(fitESEnd,esEndForEvaluation,windowType,joint, ...
        groupNames(k),phaseNames(k),n,modelMean,modelSD,abMean,abSD, ...
        rbfMean,rbfSD,maeAB,sdAbsAB,maeRBF,sdAbsRBF,signedBiasAB, ...
        signedBiasRBF,acrossSlopeR2_AB,acrossSlopeR2_RBF,timeMAE_AB, ...
        'VariableNames',{'ES_end','EvaluationES_end','EvaluationWindow', ...
        'Joint','TerrainGroup','PhaseRegion','NSlopes', ...
        'ModelPeakMean','ModelPeakSD','ABPeakMean','ABPeakSD', ...
        'RBFPeakMean','RBFPeakSD','PeakMAE_AB','PeakAbsErrorSD_AB', ...
        'PeakMAE_RBF','PeakAbsErrorSD_RBF','PeakBias_AB','PeakBias_RBF', ...
        'PeakR2AcrossSlopes_AB','PeakR2AcrossSlopes_RBF','PeakTimingMAE_AB_Pct'});
    summary = [summary;next]; %#ok<AGROW>
end
end

function [m,s] = meanStd(v)
m = mean(v,'omitnan');
s = std(v,0,'omitnan');
end
function r2 = squaredPearson(a,b)
if numel(a)<3 || std(a)==0 || std(b)==0
    r2=NaN;
    return
end
c=corrcoef(a,b);
r2=c(1,2)^2;
end
