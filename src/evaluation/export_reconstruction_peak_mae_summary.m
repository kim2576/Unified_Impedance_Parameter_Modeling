function summary = export_reconstruction_peak_mae_summary(metricsByJoint, options)
%EXPORT_RECONSTRUCTION_PEAK_MAE_SUMMARY Export reconstructed-vs-AB peak MAE.
%
% Peak definition matches evaluate_impedance_fit:
%   level/ascent : positive maximum over full stance
%   descent      : positive maximum over samples 40:100
%
% Exports:
%   reconstruction_peak_mae_per_slope.csv
%   reconstruction_peak_mae_summary.csv
%   reconstruction_peak_mae_summary.mat
%
% Summary MAE is the arithmetic mean of the per-slope absolute peak errors.

arguments
    metricsByJoint (1,1) struct
    options.SaveDirectory (1,1) string = ""
    options.ExportFiles (1,1) logical = true
    options.Verbose (1,1) logical = true
end

jointNames = ["knee","ankle"];
for joint = jointNames
    if ~isfield(metricsByJoint, char(joint))
        error('export_reconstruction_peak_mae_summary:MissingJoint', ...
            'Required metricsByJoint.%s is missing.', joint);
    end
    m = metricsByJoint.(char(joint));
    required = {'reference_slope','valid_reference_slope','peak'};
    for k = 1:numel(required)
        if ~isfield(m, required{k})
            error('export_reconstruction_peak_mae_summary:MissingMetric', ...
                '%s is missing evaluation field %s.', joint, required{k});
        end
    end
end

perSlope = buildPerSlopeTable(metricsByJoint);
summaryTable = buildSummaryTable(metricsByJoint);

summary = struct();
summary.definition = [ ...
    "Peak-vs-peak MAE across AB-reference slopes; full stance for " ...
    "level/ascent and samples 40:100 for descent."];
summary.per_slope = perSlope;
summary.table = summaryTable;
summary.source_metrics = metricsByJoint;

if options.Verbose
    fprintf('\n========== RECONSTRUCTED MOMENT PEAK MAE vs AB ==========\n');
    fprintf('Peak definition: full stance for level/ascent; samples 40:100 for descent.\n');
    disp(summaryTable);
end

if options.ExportFiles
    if strlength(options.SaveDirectory) == 0
        error('export_reconstruction_peak_mae_summary:MissingOutputDirectory', ...
            'Specify SaveDirectory when ExportFiles is true.');
    end
    if ~isfolder(options.SaveDirectory)
        mkdir(options.SaveDirectory);
    end

    writetable(perSlope, fullfile(options.SaveDirectory, ...
        'reconstruction_peak_mae_per_slope.csv'));
    writetable(summaryTable, fullfile(options.SaveDirectory, ...
        'reconstruction_peak_mae_summary.csv'));
    save(fullfile(options.SaveDirectory, ...
        'reconstruction_peak_mae_summary.mat'), 'summary');
end
end

function T = buildPerSlopeTable(metricsByJoint)
Joint = strings(0,1);
TerrainGroup = strings(0,1);
TerrainSlopeDeg = zeros(0,1);
PeakWindow = strings(0,1);
ABPeakNmKg = zeros(0,1);
ReconstructedPeakNmKg = zeros(0,1);
AbsPeakErrorNmKg = zeros(0,1);

for joint = ["knee","ankle"]
    m = metricsByJoint.(char(joint));
    slopes = m.reference_slope(:);
    valid = logical(m.valid_reference_slope(:));

    for i = 1:numel(slopes)
        if ~valid(i)
            continue
        end
        if slopes(i) < 0
            group = "Descent";
        elseif slopes(i) > 0
            group = "Ascent";
        else
            group = "Level";
        end

        Joint(end+1,1) = upperFirst(joint);
        TerrainGroup(end+1,1) = group;
        TerrainSlopeDeg(end+1,1) = slopes(i);
        PeakWindow(end+1,1) = m.peak.window_label(i);
        ABPeakNmKg(end+1,1) = m.peak.reference(i);
        ReconstructedPeakNmKg(end+1,1) = m.peak.model(i);
        AbsPeakErrorNmKg(end+1,1) = m.peak.abs_error(i);
    end
end

T = table(Joint,TerrainGroup,TerrainSlopeDeg,PeakWindow, ...
    ABPeakNmKg,ReconstructedPeakNmKg,AbsPeakErrorNmKg);
end

function T = buildSummaryTable(metricsByJoint)
Joint = strings(0,1);
TerrainGroup = strings(0,1);
NSlopes = zeros(0,1);
PeakMAE_NmKg = zeros(0,1);
PeakAbsErrorSD_NmKg = zeros(0,1);

groupNames = ["All","LW + Ascent","Ascent","Descent"];
groupFields = ["all","lw_ascent","ascent","descent"];

for joint = ["knee","ankle"]
    m = metricsByJoint.(char(joint));
    for g = 1:numel(groupNames)
        st = m.summary.(char(groupFields(g)));
        Joint(end+1,1) = upperFirst(joint);
        TerrainGroup(end+1,1) = groupNames(g);
        NSlopes(end+1,1) = st.n_slopes;
        PeakMAE_NmKg(end+1,1) = st.peak_mae.mean;
        PeakAbsErrorSD_NmKg(end+1,1) = st.peak_mae.std;
    end
end

T = table(Joint,TerrainGroup,NSlopes,PeakMAE_NmKg,PeakAbsErrorSD_NmKg);
end

function s = upperFirst(s)
s = string(s);
s = upper(extractBefore(s,2)) + extractAfter(s,1);
end
