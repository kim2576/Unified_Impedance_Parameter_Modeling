function impedance_model_full = export_impedance_model_full(jointResults, saveFile)
%{
 Save full-domain impedance surfaces for analysis.

 Converts the optimizer's internal surface convention
  71 slopes x 100 stance samples to the biomechanics-package convention
  101 stance samples x 71 slopes.

 The exported MAT file contains one struct, impedance_model_full, with:
  knee_K_stance                         [101 x 71] double
  knee_B_stance                         [101 x 71] double
  knee_theta_eq_stance                  [101 x 71] double
  knee_reconstructed_moment_stance      [101 x 71] double
  ankle_K_stance                        [101 x 71] double
  ankle_B_stance                        [101 x 71] double
  ankle_theta_eq_stance                 [101 x 71] double
  ankle_reconstructed_moment_stance     [101 x 71] double
  slope                                 [1 x 71] double
  stance_phase                          [101 x 1] double
%}

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

% Enforce the downstream comparison format explicitly.
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
% Surface input is slope x phase. interp1 operates along rows after transpose,
% producing the desired phase x slope output directly.
sourcePhase = double(sourcePhase(:));
surface = double(surface);

if size(surface,1) ~= 71 || size(surface,2) ~= numel(sourcePhase)
    error('export_impedance_model_full:UnexpectedInternalSize', ...
        'Expected an internal 71 x Nphase surface; got %dx%d.', ...
        size(surface,1), size(surface,2));
end

exported = interp1(sourcePhase, surface', exportPhase, 'linear');
end
