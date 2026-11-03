function plot_joint_ES_end_metrics(r2Table,peakTable,esEnds,joint,windowName,metricMethod,saveDir,visible)
%PLOT_JOINT_ES_END_METRICS Save separate 6-panel r2 and 2-panel peak-MAE plots.
arguments
    r2Table table
    peakTable table
    esEnds (1,:) double
    joint (1,1) string
    windowName (1,1) string
    metricMethod (1,1) string
    saveDir (1,1) string
    visible (1,1) string {mustBeMember(visible,["on","off"])} = "off"
end

cfgBaseline = get_joint_config(joint);
baselineES = cfgBaseline.phase.ES_end;
regions = ["Overall","ES","LS"];
groups = ["LW + Ascent","Descent"];
colors = [0.09 0.44 0.70; 0.86 0.35 0.20];

fig = figure('Visible',visible,'Color','w','Position',[50 80 1450 790]);
t = tiledlayout(fig,2,3,'Padding','compact','TileSpacing','compact');
for g=1:2
    for p=1:3
        ax=nexttile(t,(g-1)*3+p);
        rows=r2Table(r2Table.TerrainGroup==groups(g) & ...
            r2Table.PhaseRegion==regions(p),:);
        [m,sd]=alignToES(rows,esEnds,'MeanR2','StdR2');
        errorbar(ax,esEnds,m,sd,'-o','Color',colors(g,:), ...
            'MarkerFaceColor',colors(g,:),'LineWidth',1.6,'CapSize',7);
        title(ax,sprintf('%s | %s',groups(g),regions(p)));
        xlabel(ax,'ES end (sample index)');
        ylabel(ax,'Pearson r^2 (mean +/- SD)');
        xticks(ax,esEnds);
        xlim(ax,[min(esEnds)-2,max(esEnds)+2]);
        ylim(ax,[-0.08,1.08]);
        xline(ax,baselineES,'--','Color',[0.45 0.45 0.45], ...
            'HandleVisibility','off');
        grid(ax,'on');box(ax,'off');
    end
end
title(t,sprintf('%s reconstruction | %s windows | %s', ...
    upper(joint),windowName,metricMethod));
exportgraphics(fig,fullfile(saveDir,sprintf('%s_%s_%s_six_r2.png', ...
    joint,windowName,metricMethod)),'Resolution',220);
if visible=="off",close(fig);end

if isempty(peakTable) || metricMethod ~= "unaligned",return;end
fig=figure('Visible',visible,'Color','w','Position',[50 80 1320 540]);
t=tiledlayout(fig,1,2,'Padding','compact','TileSpacing','compact');
caseNames=["Ascent","Descent"];
for k=1:2
    ax=nexttile(t,k);
    rows=peakTable(peakTable.TerrainGroup==caseNames(k),:);
    [mAB,sdAB]=alignToES(rows,esEnds,'PeakMAE_AB','PeakAbsErrorSD_AB');
    [mRBF,sdRBF]=alignToES(rows,esEnds,'PeakMAE_RBF','PeakAbsErrorSD_RBF');
    hold(ax,'on');
    errorbar(ax,esEnds,mAB,sdAB,'-o','LineWidth',1.6, ...
        'MarkerFaceColor',colors(1,:),'Color',colors(1,:),'CapSize',7);
    errorbar(ax,esEnds,mRBF,sdRBF,'-s','LineWidth',1.6, ...
        'MarkerFaceColor',colors(2,:),'Color',colors(2,:),'CapSize',7);
    title(ax,sprintf('%s | %s peak moment',caseNames(k), ...
        string(rows.PhaseRegion(1))));
    xlabel(ax,'ES end (sample index)');
    ylabel(ax,'Peak moment MAE +/- SD (Nm/kg)');
    xticks(ax,esEnds); xlim(ax,[min(esEnds)-2,max(esEnds)+2]);
    xline(ax,baselineES,'--','Color',[0.45 0.45 0.45], ...
        'HandleVisibility','off');
    legend(ax,{'vs AB','vs RBF target'},'Location','best');
    grid(ax,'on');box(ax,'off');
end
title(t,sprintf('%s peak reconstruction | %s windows',upper(joint),windowName));
exportgraphics(fig,fullfile(saveDir,sprintf('%s_%s_peak_mae.png', ...
    joint,windowName)),'Resolution',220);
if visible=="off",close(fig);end

% The corresponding absolute peak magnitudes (not only peak MAE).
fig=figure('Visible',visible,'Color','w','Position',[50 80 1320 540]);
t=tiledlayout(fig,1,2,'Padding','compact','TileSpacing','compact');
for k=1:2
    ax=nexttile(t,k);
    rows=peakTable(peakTable.TerrainGroup==caseNames(k),:);
    hold(ax,'on');
    [mModel,sdModel]=alignToES(rows,esEnds,'ModelPeakMean','ModelPeakSD');
    [mAB,sdAB]=alignToES(rows,esEnds,'ABPeakMean','ABPeakSD');
    [mRBF,sdRBF]=alignToES(rows,esEnds,'RBFPeakMean','RBFPeakSD');
    errorbar(ax,esEnds,mModel,sdModel,'-o','LineWidth',1.5, ...
        'Color',colors(1,:),'CapSize',7);
    errorbar(ax,esEnds,mAB,sdAB,'-s','LineWidth',1.5, ...
        'Color',[0.1 0.1 0.1],'CapSize',7);
    errorbar(ax,esEnds,mRBF,sdRBF,'-^','LineWidth',1.5, ...
        'Color',colors(2,:),'CapSize',7);
    title(ax,sprintf('%s | %s peak moment',caseNames(k),rows.PhaseRegion(1)));
    xlabel(ax,'ES end (sample index)');
    ylabel(ax,'Peak moment (Nm/kg), mean +/- SD');
    xticks(ax,esEnds);xlim(ax,[min(esEnds)-2,max(esEnds)+2]);
    xline(ax,baselineES,'--','Color',[0.45 0.45 0.45], ...
        'HandleVisibility','off');
    legend(ax,{'Reconstructed','AB reference','RBF target'}, ...
        'Location','best');
    grid(ax,'on');box(ax,'off');
end
title(t,sprintf('%s peak magnitudes | %s windows',upper(joint),windowName));
exportgraphics(fig,fullfile(saveDir,sprintf('%s_%s_peak_magnitude.png', ...
    joint,windowName)),'Resolution',220);
if visible=="off",close(fig);end
end

function [m,sd] = alignToES(rows,esEnds,meanField,sdField)
m=NaN(size(esEnds));sd=m;
for c=1:numel(esEnds)
    idx=find(rows.ES_end==esEnds(c),1);
    if ~isempty(idx)
        m(c)=rows.(meanField)(idx);
        sd(c)=rows.(sdField)(idx);
    end
end
end
