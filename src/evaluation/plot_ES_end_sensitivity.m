function fig = plot_ES_end_sensitivity(detailTable, ES_end_values, outputFile, subtitle, visible)
% PLOT_ES_END_SENSITIVITY Six panels (2 terrain groups x 3 intervals).
% Each panel compares the knee and ankle mean +/- 1 SD Pearson r^2
% across the supplied candidate ES_end sample indices.
arguments
    detailTable table
    ES_end_values (1,:) double
    outputFile (1,1) string
    subtitle (1,1) string
    visible (1,1) string {mustBeMember(visible,["on","off"])} = "off"
end

groups = ["LW + Ascent", "Descent"];
phases = ["Overall", "ES", "LS"];
joints = ["Knee", "Ankle"];
colors = [0.12 0.43 0.72; 0.85 0.36 0.15];
markers = {'o','s'};

fig = figure('Color','w', 'Visible',visible, ...
    'Position',[80 80 1550 900], 'Name','ES-end reconstruction sensitivity');
tl = tiledlayout(fig,2,3, 'TileSpacing','compact','Padding','compact');
legendHandles = gobjects(1,2);
for g = 1:2
    for p = 1:3
        ax = nexttile(tl,(g-1)*3+p);
        hold(ax,'on');
        for j = 1:2
            subset = detailTable( ...
                detailTable.Joint == joints(j) & ...
                detailTable.TerrainGroup == groups(g) & ...
                detailTable.PhaseRegion == phases(p),:);
            means = NaN(size(ES_end_values));
            stds = NaN(size(ES_end_values));
            ns = zeros(size(ES_end_values));
            for c = 1:numel(ES_end_values)
                r = find(subset.ES_end == ES_end_values(c), 1);
                if ~isempty(r)
                    means(c) = subset.MeanR2(r);
                    stds(c) = subset.StdR2(r);
                    ns(c) = subset.NSlopes(r);
                end
            end
            h = errorbar(ax, ES_end_values, means, stds, ...
                'LineStyle','-', 'Marker',markers{j}, ...
                'LineWidth',1.5, 'MarkerSize',6, 'CapSize',7, ...
                'Color',colors(j,:), 'MarkerFaceColor',colors(j,:));
            if g==1 && p==1
                legendHandles(j)=h;
            end
            if any(ns > 0) && numel(unique(ns(ns>0))) > 1
                warning('plot_ES_end_sensitivity:ChangingSlopeCount', ...
                    'Valid reference-slope count varies for %s, %s, %s.', ...
                    joints(j), groups(g), phases(p));
            end
        end
        title(ax, sprintf('%s | %s', groups(g), phases(p)), ...
            'FontWeight','bold');
        xlabel(ax,'cfg.phase.ES_end (sample index)');
        ylabel(ax,'Squared Pearson r^2 (mean +/- 1 SD)');
        xticks(ax,ES_end_values);
        xlim(ax,[min(ES_end_values)-2,max(ES_end_values)+2]);
        ylim(ax,[-0.1,1.1]);
        grid(ax,'on');
        box(ax,'off');
    end
end
legend(legendHandles,cellstr(joints),'Orientation','horizontal', ...
    'Location','southoutside');
title(tl, sprintf('Stance-moment reconstruction vs. ES end | %s',subtitle), ...
    'FontWeight','bold', 'FontSize',14);

if ~isfolder(fileparts(outputFile))
    mkdir(fileparts(outputFile));
end
exportgraphics(fig,outputFile,'Resolution',220);
if visible == "off"
    close(fig);
end
end
