%{
Parameterize impedance using predefined configuration.
%}
clear;
close all;
clc;

project_root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(project_root, 'src')));

rbf_model_file = fullfile(project_root, 'data', 'RBF_models.mat');
ab_reference_file = fullfile(project_root, 'data', 'AB_refs.mat');
output_dir = fullfile(project_root, 'outputs', 'Plots');
evaluation_dir = fullfile(project_root, 'outputs', 'Evaluation');

% Select which preprocessed/RBF-unified angular-velocity trajectory to use:
%   "original" : <joint>_angle_dot_stance (default, measured/preprocessed)
%   "norm"     : <joint>_angle_dot_stance_norm (derived from averaged angles
%                upstream, then modeled using the biomechanics package)
% Both use the RBF surface supplied in data/RBF_models.mat. The impedance
% package no longer calculates angular velocity using diff(joint angle).
angular_velocity_variant = "original";

inputs = load_impedance_inputs(rbf_model_file, ab_reference_file, ...
    AngularVelocityVariant=angular_velocity_variant);
fprintf('Angular velocity input: %s (RBF-unified preprocessed velocities)\n', ...
    angular_velocity_variant);

joint_metrics = struct();

for joint = ["knee", "ankle"]
    cfg = get_joint_config(joint);
    data = select_joint_biomechanics(inputs, joint);
    equilibrium = build_equilibrium_angle(data, cfg);
    bounds = build_impedance_bounds(inputs.slope, cfg);

    %% Explicit legacy-style arctangent upper-bound plots
    % The bound construction itself is performed in build_impedance_bounds().
    % Here, reconstruct and plot the fitted arctangent curves explicitly so
    % the slope-dependent K/B envelopes are directly visible in this script.
    %
    % Legacy form:
    %   y = p1 * atan(p2 * (s - p3)) + p4
    % K uses the fitted curve directly plus its offset.
    % B reverses the fitted curve before applying its offset.

    % ----- K upper-bound arctangent -----
    model_K = @(p, s) p(1) .* atan(p(2) .* (s - p(3))) + p(4);
    x_fit_K = inputs.slope;
    y_fit_K = model_K(bounds.fit.K.params, x_fit_K);
    K_upper_const_mat = y_fit_K + bounds.fit.K.offset;

    fig_K_bound = figure( ...
        'Name', upper(joint) + " K upper-bound arctangent", ...
        'Visible', 'on');
    plot(x_fit_K, K_upper_const_mat, 'g-', 'LineWidth', 5);
    set(gca, 'FontSize', 16);
    xlim([-35 35])
    grid off;

    % ----- B upper-bound arctangent -----
    model_B = @(p, s) p(1) .* atan(p(2) .* (s - p(3))) + p(4);
    x_fit_B = inputs.slope;
    y_fit_B = -model_B(bounds.fit.B.params, x_fit_B);
    B_upper_const_mat = y_fit_B + bounds.fit.B.offset;

    fig_B_bound = figure( ...
        'Name', upper(joint) + " B upper-bound arctangent", ...
        'Visible', 'on');
    plot(x_fit_B, B_upper_const_mat, 'g-', 'LineWidth', 5);
    set(gca, 'FontSize', 16);
    xlim([-35 35])
    ylim([0.0005 0.004])
    grid off;

    if ~isfolder(output_dir)
        mkdir(output_dir);
    end
    exportgraphics(fig_K_bound, ...
        fullfile(output_dir, lower(joint) + "_K_upper_bound_arctan.png"), ...
        'Resolution', 200);
    exportgraphics(fig_B_bound, ...
        fullfile(output_dir, lower(joint) + "_B_upper_bound_arctan.png"), ...
        'Resolution', 200);

    early = fit_early_stance_impedance(data, equilibrium, bounds, cfg);
    late = fit_late_stance_impedance(data, equilibrium, bounds, early, cfg);

    result = assemble_full_stance_impedance( ...
        data, equilibrium, bounds, early, late, cfg);
    metrics = evaluate_impedance_fit(result);
    % Store per-slope reconstruction metrics for the Table I-style summary.
    joint_metrics.(char(joint)) = metrics;

    fprintf('\n=== %s FULL-STANCE MODEL ===\n', upper(joint));
    fprintf('Returned continuous slopes:    %d / %d\n', ...
        nnz(result.success), numel(result.success));
    fprintf('Evaluated AB-reference slopes: %d / %d\n', ...
        nnz(metrics.valid_reference_slope), numel(metrics.valid_reference_slope));
    fprintf('Moment RMSE:                   %.5f +/- %.5f Nm/kg\n', ...
        metrics.summary.all.rmse_total.mean, metrics.summary.all.rmse_total.std);
    fprintf('Moment MAE:                    %.5f +/- %.5f Nm/kg\n', ...
        metrics.summary.all.mae_total.mean, metrics.summary.all.mae_total.std);
    fprintf('R^2:                           %.5f +/- %.5f\n', ...
        metrics.summary.all.r2_total.mean, metrics.summary.all.r2_total.std);
    fprintf('Phase-aligned ES R^2:          %.5f +/- %.5f\n', ...
        metrics.summary.all.r2_phase_aligned_early.mean, ...
        metrics.summary.all.r2_phase_aligned_early.std);
    fprintf('Phase-aligned LS R^2:          %.5f +/- %.5f\n', ...
        metrics.summary.all.r2_phase_aligned_late.mean, ...
        metrics.summary.all.r2_phase_aligned_late.std);

    if any(result.success)
        fprintf('Worst ES->LS K discontinuity:  %.3e\n', ...
            max(abs(result.continuity.K(result.success)), [], 'omitnan'));
        fprintf('Worst ES->LS B discontinuity:  %.3e\n', ...
            max(abs(result.continuity.B(result.success)), [], 'omitnan'));
    end

    if any(~result.success)
        fprintf('Missing solver-output slopes:  %s deg\n', mat2str(result.failed_slope));
    end

    % Full-range reproduction is required to match the legacy scripts.
    assert(result.full_range_available, ...
        'A solver returned no coefficient vector at one or more %s slopes.', joint);

    % The plots use every returned lsqlin solution across the full slope range.
    plot_impedance_results(result, metrics, ...
        SaveDirectory=output_dir, ...
        Visible="on");
end

% Summarize six values per joint: LW + ascent / descent x overall / ES / LS.
% Overall always uses direct squared Pearson correlation. The primary
% table uses phase-aligned ES/LS R^2 (matching the existing evaluator's
% reported regional metric); an unaligned companion table is also saved.
r2_summary = export_reconstruction_r2_summary(joint_metrics, ...
    SaveDirectory=evaluation_dir, ...
    VelocityVariant=angular_velocity_variant, ...
    Visible="on");

fprintf('\nSaved six-column reconstruction tables to:\n  %s\n', evaluation_dir);

fprintf(['\nInspect the saved independent figures in:\n  %s\n' ...
    'Do not proceed to export/controller use if the surfaces show unexpected gaps, discontinuities, or nonphysical parameter trends.\n'], ...
    output_dir);
