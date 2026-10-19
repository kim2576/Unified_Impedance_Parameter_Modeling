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

inputs = load_impedance_inputs(rbf_model_file, ab_reference_file);

for joint = ["knee", "ankle"]
    cfg = get_joint_config(joint);
    data = select_joint_biomechanics(inputs, joint);
    equilibrium = build_equilibrium_angle(data, cfg);
    bounds = build_impedance_bounds(inputs.slope, cfg);
    early = fit_early_stance_impedance(data, equilibrium, bounds, cfg);
    late = fit_late_stance_impedance(data, equilibrium, bounds, early, cfg);

    result = assemble_full_stance_impedance( ...
        data, equilibrium, bounds, early, late, cfg);
    metrics = evaluate_impedance_fit(result);

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

fprintf(['\nInspect the saved independent figures in:\n  %s\n' ...
    'Do not proceed to export/controller use if the surfaces show unexpected gaps, discontinuities, or nonphysical parameter trends.\n'], ...
    output_dir);
