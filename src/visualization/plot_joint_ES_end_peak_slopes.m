function plot_joint_ES_end_peak_slopes(perSlope,esEnds,joint,windowName,saveDir,visible)
%PLOT_JOINT_ES_END_PEAK_SLOPES Show peak moment versus AB slope for each candidate.
arguments
    perSlope table
    esEnds (1,:) double
    joint (1,1) string
    windowName (1,1) string
    saveDir (1,1) string
    visible (1,1) string {mustBeMember(visible,["on","off"])} = "off"
end
fig=figure('Visible',visible,'Color','w','Position',[60 80 1320 520]);
t=tiledlayout(fig,1,2,'Padding','compact','TileSpacing','compact');
labels=["Ascent","Descent"];
palette=lines(numel(esEnds));
for k=1:2
    ax=nexttile(t,k);hold(ax,'on');
    subset=perSlope(perSlope.TerrainGroup==labels(k),:);
    for c=1:numel(esEnds)
        z=subset(subset.ES_end==esEnds(c),:);
        [sl,sortidx]=sort(z.SlopeDeg);
        plot(ax,sl,z.ModelPeakNmKg(sortidx),'-o','LineWidth',1.3, ...
            'Color',palette(c,:),'DisplayName',sprintf('Model ES=%d',esEnds(c)));
        if windowName=="native"
            % The AB peak changes when the ES/LS window changes: plot the
            % actual matching AB reference for EVERY candidate.
            plot(ax,sl,z.ABPeakNmKg(sortidx),'--','LineWidth',0.95, ...
                'Color',palette(c,:),'DisplayName',sprintf('AB ES=%d',esEnds(c)));
        end
    end
    baseline=subset(subset.ES_end==esEnds(1),:);
    if windowName=="fixed"
        [sl,sortidx]=sort(baseline.SlopeDeg);
        plot(ax,sl,baseline.ABPeakNmKg(sortidx),'k--d','LineWidth',2, ...
            'DisplayName','AB reference (fixed window)');
    end
    if windowName=="native"
        title(ax,sprintf('%s | %s peaks (window varies)',labels(k),baseline.PhaseRegion(1)));
    else
        title(ax,sprintf('%s | %s peaks (fixed window)',labels(k),baseline.PhaseRegion(1)));
    end
    xlabel(ax,'AB terrain slope (deg)');ylabel(ax,'Peak moment (Nm/kg)');
    grid(ax,'on');box(ax,'off');
    if k==2
        legend(ax,'Location','bestoutside');
    end
end
title(t,sprintf('%s peak moment by slope | %s evaluation',upper(joint),windowName));
exportgraphics(fig,fullfile(saveDir,sprintf('%s_%s_peak_by_slope.png', ...
    joint,windowName)),'Resolution',220);
if visible=="off",close(fig);end
end
