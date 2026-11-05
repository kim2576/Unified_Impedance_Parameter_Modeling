function figs = plot_impedance_results(result, metrics, options)
% Plot reconstructed moment and impedance surfaces.

arguments
    result (1,1) struct
    metrics (1,1) struct
    options.SaveDirectory (1,1) string = ""
    options.Visible (1,1) string {mustBeMember(options.Visible,["on","off"])} = "on"
end

jointLabel = upperFirst(result.joint);
phase = result.phase;
slope = result.slope;

% Joint-specific figure geometry.
if result.joint == "knee"
    momentPosition = [100 100 1000 500];
    momentView = [35 35];

    kPosition = [100 100 500 500];
    kView = [25 25];

    bPosition = [100 100 500 500];
    bView = [25 25];

    thetaPosition = [100 100 1000 600];
    thetaView = [35 45];
else
    momentPosition = [100 100 1000 500];
    momentView = [35 35];

    kPosition = [100 100 500 500];
    kView = [25 30];

    bPosition = [100 100 500 500];
    bView = [25 30];

    thetaPosition = [100 100 1000 500];
    thetaView = [25 45];
end

figs = struct();

%% Reconstructed moment
figs.moment = figure( ...
    'Name', jointLabel + " reconstructed moment", ...
    'Visible', options.Visible, ...
    'Position', momentPosition);
ax = axes(figs.moment);

scatterReferenceMoment(ax, result);
hold(ax,'on');

surf(ax, slope, phase, result.moment', ...
    'FaceAlpha',0.5, ...
    'EdgeAlpha',0.5);

xlabel(ax,'Terrain slope (deg)');
ylabel(ax,'Stance phase (%)');
zlabel(ax,'Moment (Nm/kg)');
title(ax, sprintf('%s reconstructed moment | MAE %.3f +/- %.3f Nm/kg', ...
    jointLabel, metrics.summary.all.mae_total.mean, ...
    metrics.summary.all.mae_total.std));
set(ax,'FontSize',20);
grid(ax,'off');
view(ax,momentView(1),momentView(2));

%% Stiffness K
figs.K = figure( ...
    'Name', jointLabel + " stiffness K", ...
    'Visible', options.Visible, ...
    'Position', kPosition);
ax = axes(figs.K);

surf(ax, slope, phase, result.K', ...
    'FaceAlpha',0.5, ...
    'EdgeAlpha',0.5);
hold(ax,'on');

% Arctangent upper envelope at 50% stance.
plot3(ax, ...
    slope, ...
    50 * ones(size(slope)), ...
    result.bounds.K.upper, ...
    'g--', ...
    'LineWidth',5);

xlabel(ax,'Terrain slope (deg)');
ylabel(ax,'Stance phase (%)');
zlabel(ax,'K (Nm/kg/deg)');
title(ax, jointLabel + " stiffness K");
set(ax,'FontSize',20);
grid(ax,'off');
view(ax,kView(1),kView(2));

%% Damping B
figs.B = figure( ...
    'Name', jointLabel + " damping B", ...
    'Visible', options.Visible, ...
    'Position', bPosition);
ax = axes(figs.B);

surf(ax, slope, phase, result.B', ...
    'FaceAlpha',0.5, ...
    'EdgeAlpha',0.5);
hold(ax,'on');

% Arctangent upper envelope at 50% stance.
plot3(ax, ...
    slope, ...
    50 * ones(size(slope)), ...
    result.bounds.B.upper, ...
    'g--', ...
    'LineWidth',5);

xlabel(ax,'Terrain slope (deg)');
ylabel(ax,'Stance phase (%)');
zlabel(ax,'B (Nms/kg/deg)');
title(ax, jointLabel + " damping B");
set(ax,'FontSize',20);
grid(ax,'off');
view(ax,bView(1),bView(2));

%% Equilibrium angle
figs.equilibrium = figure( ...
    'Name', jointLabel + " equilibrium angle", ...
    'Visible', options.Visible, ...
    'Position', thetaPosition);
ax = axes(figs.equilibrium);

surf(ax, slope, phase, result.equilibrium_angle', ...
    'FaceAlpha',0.5, ...
    'EdgeAlpha',0.5);

xlabel(ax,'Terrain slope (deg)');
ylabel(ax,'Stance phase (%)');
zlabel(ax,'Equilibrium angle (deg)');
title(ax, jointLabel + " equilibrium angle \theta_{eq}");
set(ax,'FontSize',20);
grid(ax,'off');
view(ax,thetaView(1),thetaView(2));

%% Save figures
if strlength(options.SaveDirectory) > 0
    if ~isfolder(options.SaveDirectory)
        mkdir(options.SaveDirectory);
    end

    jointName = lower(result.joint);

    exportgraphics(figs.moment, ...
        fullfile(options.SaveDirectory, jointName + "_reconstructed_moment.png"), ...
        'Resolution',200);

    exportgraphics(figs.K, ...
        fullfile(options.SaveDirectory, jointName + "_stiffness_K.png"), ...
        'Resolution',200);

    exportgraphics(figs.B, ...
        fullfile(options.SaveDirectory, jointName + "_damping_B.png"), ...
        'Resolution',200);

    exportgraphics(figs.equilibrium, ...
        fullfile(options.SaveDirectory, jointName + "_equilibrium_angle.png"), ...
        'Resolution',200);
end
end

function scatterReferenceMoment(ax, result)
refSlope = result.reference.slope;
refPhase = result.reference.phase(1:100);
refMoment = result.reference.moment(:,1:100);
[S,P] = meshgrid(refSlope, refPhase);
Z = refMoment';

scatter3(ax, S(:), P(:), Z(:), 4, 'filled', ...
    'MarkerFaceColor','red', ...
    'MarkerEdgeColor','red', ...
    'MarkerFaceAlpha',0.2, ...
    'DisplayName','AB reference');
end

function label = upperFirst(textValue)
chars = char(textValue);
chars(1) = upper(chars(1));
label = string(chars);
end
