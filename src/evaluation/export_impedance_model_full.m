function impedance_model_full = export_impedance_model_full(jointResults, saveFile)
%EXPORT_IMPEDANCE_MODEL_FULL Export 101 x 71 phase-by-slope surfaces.

arguments
    jointResults (1,1) struct
    saveFile (1,1) string
end

requiredJoints = ["knee", "ankle"];
for joint = requiredJoints
    if ~isfield(jointResults, char(joint))
        error('export_impedance_model_full:MissingJoint', ...
            'Missing jointResults.%s.', joint);
    end
end

knee = jointResults.knee;
ankle = jointResults.ankle;

if ~isequal(knee.slope, ankle.slope)
    error('export_impedance_model_full:SlopeMismatch', ...
        'Knee and ankle terrain-slope domains must match.');
end

exportPhase = linspace(0, 100, 101)';

impedance_model_full = struct();
impedance_model_full.knee_K_stance = resampleSurface(knee.phase, knee.K, exportPhase);
impedance_model_full.knee_B_stance = resampleSurface(knee.phase, knee.B, exportPhase);
impedance_model_full.knee_theta_eq_stance = ...
    resampleSurface(knee.phase, knee.equilibrium_angle, exportPhase);
impedance_model_full.knee_reconstructed_moment_stance = ...
    resampleSurface(knee.phase, knee.moment, exportPhase);

impedance_model_full.ankle_K_stance = resampleSurface(ankle.phase, ankle.K, exportPhase);
impedance_model_full.ankle_B_stance = resampleSurface(ankle.phase, ankle.B, exportPhase);
impedance_model_full.ankle_theta_eq_stance = ...
    resampleSurface(ankle.phase, ankle.equilibrium_angle, exportPhase);
impedance_model_full.ankle_reconstructed_moment_stance = ...
    resampleSurface(ankle.phase, ankle.moment, exportPhase);

impedance_model_full.slope = double(knee.slope(:)');
impedance_model_full.stance_phase = double(exportPhase);

% Phase-by-slope output used by downstream comparisons.
surfaceFields = [ ...
    "knee_K_stance", ...
    "knee_B_stance", ...
    "knee_theta_eq_stance", ...
    "knee_reconstructed_moment_stance", ...
    "ankle_K_stance", ...
    "ankle_B_stance", ...
    "ankle_theta_eq_stance", ...
    "ankle_reconstructed_moment_stance"];

for field = surfaceFields
    value = impedance_model_full.(char(field));
    if ~isa(value, 'double') || ~isequal(size(value), [101, 71])
        error('export_impedance_model_full:UnexpectedSurfaceSize', ...
            '%s must be a 101x71 double matrix; got %dx%d %s.', ...
            field, size(value,1), size(value,2), class(value));
    end
end

saveFolder = fileparts(saveFile);
if strlength(saveFolder) > 0 && ~isfolder(saveFolder)
    mkdir(saveFolder);
end

save(char(saveFile), 'impedance_model_full', '-v7');
end

function exported = resampleSurface(sourcePhase, surface, exportPhase)
% Convert slope-by-phase input to phase-by-slope output.
sourcePhase = double(sourcePhase(:));
surface = double(surface);

if size(surface,1) ~= 71 || size(surface,2) ~= numel(sourcePhase)
    error('export_impedance_model_full:UnexpectedInternalSize', ...
        'Expected an internal 71 x Nphase surface; got %dx%d.', ...
        size(surface,1), size(surface,2));
end

exported = interp1(sourcePhase, surface', exportPhase, 'linear');
end
