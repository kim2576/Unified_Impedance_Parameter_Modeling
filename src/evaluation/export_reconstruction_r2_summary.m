function summary = export_reconstruction_r2_summary(metricsByJoint, options)
%EXPORT_RECONSTRUCTION_R2_SUMMARY Reproduce the paper-style six-metric table.
%
% Six metrics per joint: LW + Ascent and Descent, each Overall / ES / LS.
% Values are mean +/- 1 sample SD of slope-wise squared Pearson r values.
% Each AB reference terrain slope is weighted equally (not every phase
% sample, stride or participant). Level walking (0 deg) belongs only to
% the LW + Ascent group, and negative slopes belong to Descent.
%
% Primary "phase_aligned" summary: Overall = unaligned; ES/LS = the
% existing evaluator's cross-correlation phase-aligned regional R^2.
% Companion "unaligned" summary: Overall, ES and LS are all unaligned.
% Both versions are labeled separately to avoid an ambiguous table.
%
% Exports: each variant as 2x6 human-readable CSV and matching PNG;
% both variants' 12-row numeric long tables; per-slope numeric CSV;
% and a MAT summary containing the input evaluation metrics.

arguments
    metricsByJoint (1,1) struct
    options.SaveDirectory (1,1) string = ""
    options.Visible (1,1) string {mustBeMember(options.Visible, ["on","off"])} = "on"
    options.VelocityVariant (1,1) string = "unspecified"
    options.ExportFiles (1,1) logical = true
    options.Verbose (1,1) logical = true
end

jointNames = ["knee", "ankle"];
for j = 1:numel(jointNames)
    name = char(jointNames(j));
    if ~isfield(metricsByJoint, name)
        error('export_reconstruction_r2_summary:MissingJoint', ...
            'Required metricsByJoint.%s is missing.', name);
    end
    metrics = metricsByJoint.(name);
    required = {'reference_slope','valid_reference_slope','r2','r2_phase_aligned'};
    for k = 1:numel(required)
        if ~isfield(metrics,required{k})
            error('export_reconstruction_r2_summary:MissingMetric', ...
                '%s is missing evaluation field %s.', name,required{k});
        end
    end
    n = numel(metrics.reference_slope);
    fieldsToCheck = {metrics.valid_reference_slope, ...
        metrics.r2.total, metrics.r2.early,metrics.r2.late, ...
        metrics.r2_phase_aligned.early, metrics.r2_phase_aligned.late};
    if any(cellfun(@numel, fieldsToCheck) ~= n)
        error('export_reconstruction_r2_summary:SizeMismatch', ...
            '%s metric vectors must match the number of reference slopes.',name);
    end
end

summary = struct();
summary.angular_velocity_variant = options.VelocityVariant;
summary.group_definition.LW_Ascent = 'reference slopes >= 0 degrees (including level walking)';
summary.group_definition.Descent = 'reference slopes < 0 degrees';
summary.statistic = 'Arithmetic mean and sample SD of per-slope Pearson correlation squared; NaNs omitted';
summary.phase_aligned = summarizeVariant(metricsByJoint, true);
summary.unaligned = summarizeVariant(metricsByJoint, false);
summary.per_slope = constructPerSlopeTable(metricsByJoint);
summary.source_metrics = metricsByJoint;

if options.Verbose
fprintf('\n========== MOMENT RECONSTRUCTION R^2 (mean +/- 1 SD) ==========\n');
fprintf('Angular-velocity input: %s\n', options.VelocityVariant);
fprintf('Overall: unaligned Pearson r^2. Primary ES/LS: phase-aligned r^2.\n');
fprintf('LW+Ascent includes 0 degrees; Descent is < 0 degrees.\n');
disp(summary.phase_aligned.wide);
fprintf('Companion: unaligned R^2 across overall, ES, and LS.\n');
disp(summary.unaligned.wide);
end

if options.ExportFiles
    if strlength(options.SaveDirectory) == 0
        error('export_reconstruction_r2_summary:MissingOutputDirectory', ...
            'Specify SaveDirectory when ExportFiles is true.');
    end
    if ~isfolder(options.SaveDirectory)
        mkdir(options.SaveDirectory);
    end
    saveRoot = options.SaveDirectory;
    writetable(summary.phase_aligned.wide, ...
        fullfile(saveRoot,'reconstruction_r2_summary_phase_aligned.csv'));
    writetable(summary.unaligned.wide, ...
        fullfile(saveRoot,'reconstruction_r2_summary_unaligned.csv'));
    writetable(summary.phase_aligned.detail, ...
        fullfile(saveRoot,'reconstruction_r2_numeric_phase_aligned.csv'));
    writetable(summary.unaligned.detail, ...
        fullfile(saveRoot,'reconstruction_r2_numeric_unaligned.csv'));
    writetable(summary.per_slope, ...
        fullfile(saveRoot,'reconstruction_r2_per_slope.csv'));
    save(fullfile(saveRoot,'reconstruction_r2_summary.mat'),'summary');

    plotPaperTable(summary.phase_aligned, ...
        'Overall: unaligned r^2 | ES/LS: phase-aligned r^2', ...
        fullfile(saveRoot,'reconstruction_r2_summary_phase_aligned.png'), ...
        options.Visible);
    plotPaperTable(summary.unaligned, ...
        'Overall, ES, LS: unaligned r^2', ...
        fullfile(saveRoot,'reconstruction_r2_summary_unaligned.png'), ...
        options.Visible);
end
end

function out = summarizeVariant(metricsByJoint, phaseAligned)
jointNames = ["knee","ankle"];
groupNames = ["LW + Ascent","Descent"];
phaseNames = ["Overall","ES","LS"];
headers = { 'LW_Ascent_Overall','LW_Ascent_ES','LW_Ascent_LS', ...
    'Descent_Overall','Descent_ES','Descent_LS' };

out = struct();
out.means = NaN(2,6);
out.stds = NaN(2,6);
out.n = zeros(2,6);
out.formatted = strings(2,6);

% Long-form numerical table: 2 joints * 2 slope groups * 3 regions = 12 rows.
rowJoint = strings(12,1);
rowGroup = strings(12,1);
rowPhase = strings(12,1);
rowMethod = strings(12,1);
rowN = zeros(12,1);
rowMean = NaN(12,1);
rowStd = NaN(12,1);
rowFormatted = strings(12,1);
r = 0;
for j = 1:2
    metrics = metricsByJoint.(char(jointNames(j)));
    slopes = metrics.reference_slope(:)';
    valid = logical(metrics.valid_reference_slope(:)');
    for g = 1:2
        if g == 1
            groupMask = valid & (slopes >= 0);
        else
            groupMask = valid & (slopes < 0);
        end
        for p = 1:3
            col = (g-1)*3+p;
            switch p
                case 1
                    values = metrics.r2.total(:)';
                    method = "unaligned";
                case 2
                    if phaseAligned
                        values = metrics.r2_phase_aligned.early(:)';
                        method = "phase_aligned";
                    else
                        values = metrics.r2.early(:)';
                        method = "unaligned";
                    end
                case 3
                    if phaseAligned
                        values = metrics.r2_phase_aligned.late(:)';
                        method = "phase_aligned";
                    else
                        values = metrics.r2.late(:)';
                        method = "unaligned";
                    end
            end
            selected = values(groupMask & isfinite(values));
            out.n(j,col) = numel(selected);
            if ~isempty(selected)
                out.means(j,col) = mean(selected);
                out.stds(j,col) = std(selected,0); % 1 sample SD across slopes
                out.formatted(j,col) = sprintf('%.2f %c %.2f', ...
                    out.means(j,col),char(177),out.stds(j,col));
            else
                out.formatted(j,col) = "N/A";
            end
            r = r + 1;
            rowJoint(r) = upperFirst(jointNames(j));
            rowGroup(r) = groupNames(g);
            rowPhase(r) = phaseNames(p);
            rowMethod(r) = method;
            rowN(r) = out.n(j,col);
            rowMean(r) = out.means(j,col);
            rowStd(r) = out.stds(j,col);
            rowFormatted(r) = out.formatted(j,col);
        end
    end
end

out.detail = table(rowJoint,rowGroup,rowPhase,rowMethod,rowN,rowMean,rowStd,rowFormatted, ...
    'VariableNames', {'Joint','TerrainGroup','PhaseRegion','R2Method', ...
                      'NSlopes','MeanR2','StdR2','MeanPlusMinusStd'});
out.wide = table(["Knee";"Ankle"], 'VariableNames',{'Joint'});
for c = 1:6
    out.wide.(headers{c}) = out.formatted(:,c);
end
out.method = "unaligned";
if phaseAligned
    out.method = "phase_aligned_ES_LS";
end
end

function tableOut = constructPerSlopeTable(metricsByJoint)
J = strings(0,1);
S = zeros(0,1);
V = false(0,1);
RAll = zeros(0,1);
RES = zeros(0,1);
RLS = zeros(0,1);
RAES = zeros(0,1);
RALS = zeros(0,1);
ShiftES = zeros(0,1);
ShiftLS = zeros(0,1);
for joint = ["knee","ankle"]
    m = metricsByJoint.(char(joint));
    n = numel(m.reference_slope);
    J = [J; repmat(upperFirst(joint),n,1)]; %#ok<AGROW>
    S = [S; m.reference_slope(:)]; %#ok<AGROW>
    V = [V; logical(m.valid_reference_slope(:))]; %#ok<AGROW>
    RAll = [RAll; m.r2.total(:)]; %#ok<AGROW>
    RES = [RES; m.r2.early(:)]; %#ok<AGROW>
    RLS = [RLS; m.r2.late(:)]; %#ok<AGROW>
    RAES = [RAES; m.r2_phase_aligned.early(:)]; %#ok<AGROW>
    RALS = [RALS; m.r2_phase_aligned.late(:)]; %#ok<AGROW>
    if isfield(m,'phase_shift_samples')
        ShiftES = [ShiftES; m.phase_shift_samples.early(:)]; %#ok<AGROW>
        ShiftLS = [ShiftLS; m.phase_shift_samples.late(:)]; %#ok<AGROW>
    else
        ShiftES = [ShiftES; NaN(n,1)]; %#ok<AGROW>
        ShiftLS = [ShiftLS; NaN(n,1)]; %#ok<AGROW>
    end
end
tableOut = table(J,S,V,RAll,RES,RLS,RAES,RALS,ShiftES,ShiftLS, ...
    'VariableNames',{'Joint','TerrainSlopeDeg','ValidReference', ...
    'OverallR2','ESR2Unaligned','LSR2Unaligned', ...
    'ESR2PhaseAligned','LSR2PhaseAligned','ESShiftSamples','LSShiftSamples'});
end

function plotPaperTable(st, subtitle, outputFile, visible)
% Figure composed of plotted rectangles/text rather than uitable so
% exportgraphics exports it consistently as a regular MATLAB figure.
fig = figure('Color','w','Visible',visible,'Position',[100 100 1300 850], ...
    'Name','Moment reconstruction r^2 summary');
ax = axes('Parent',fig,'Units','normalized','Position',[0.03 0.04 0.94 0.92]);
hold(ax,'on');
axis(ax,[0 1 0 1]); axis(ax,'off');
headColor = [25 95 127]/255;
borderColor = [0 0 0];
firstWidth = 0.20;
cellWidth = (1-firstWidth)/3;
headers = { 'Overall','ES','LS' };
% Group header, knee row, ankle row; then repeat for descent.
segments = { ...
    [0.80 0.96], [0.65 0.80], [0.50 0.65], ...
    [0.34 0.50], [0.19 0.34], [0.04 0.19] };
for group = 1:2
    if group == 1
        h = 1; row1 = 2; row2 = 3;
        groupLabel = 'LW + Ascent';
    else
        h = 4; row1 = 5; row2 = 6;
        groupLabel = 'Descent';
    end
    yr = segments{h};
    drawCell(ax,0,yr(1),firstWidth,diff(yr),headColor,'',14,true,borderColor);
    for p = 1:3
        col = (group-1)*3+p;
        x = firstWidth+(p-1)*cellWidth;
        % 2-line header with condition and phase label.
        drawCell(ax,x,yr(1),cellWidth,diff(yr),headColor, ...
            sprintf('%s\n%s',groupLabel,headers{p}),18,true,borderColor);
    end
    for j = 1:2
        rr = row1;
        if j==2, rr=row2; end
        yr = segments{rr};
        if j==1, jointName='Knee'; else, jointName='Ankle'; end
        drawCell(ax,0,yr(1),firstWidth,diff(yr),[1 1 1],jointName,19,true,borderColor);
        for p=1:3
            col = (group-1)*3+p;
            drawCell(ax,firstWidth+(p-1)*cellWidth,yr(1),cellWidth,diff(yr), ...
                [1 1 1],char(st.formatted(j,col)),20,false,borderColor);
        end
    end
end
% Clearly identify the R^2 definition; the six statistics are across slopes.
text(ax,0.5,0.011,sprintf('Mean %c 1 SD across AB-reference slopes. %s',char(177),subtitle), ...
    'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'FontSize',10,'Color',[0.25 0.25 0.25], 'Interpreter','none');
exportgraphics(fig,outputFile,'Resolution',200);
if visible == "off"
    close(fig);
end
end

function drawCell(ax,x,y,w,h,bg,label,fontSize,boldFlag,borderColor)
rectangle(ax,'Position',[x y w h],'FaceColor',bg, ...
    'EdgeColor',borderColor,'LineWidth',1.6);
if isempty(label), return; end
if isequal(bg,[1 1 1])
    tc=[0 0 0];
else
    tc=[1 1 1];
end
weight = 'normal';
if boldFlag, weight='bold'; end
text(ax,x+w/2,y+h/2,label, ...
    'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'Color',tc,'FontSize',fontSize,'FontWeight',weight, ...
    'Interpreter','none');
end

function label = upperFirst(name)
label=char(name);
label(1)=upper(label(1));
label=string(label);
end
