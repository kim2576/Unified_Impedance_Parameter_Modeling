function inputs = load_impedance_inputs(rbfModelFile, abReferenceFile, options)
%{
Load the outputs of the biomechanics-modeling package.

inputs = load_impedance_inputs(rbfModelFile, abReferenceFile, AngularVelocityVariant=...)

Expected MAT-file variables:
   rbfModelFile   : RBF_model_full
   abReferenceFile: target_biomechanics_total

The returned struct deliberately preserves the upstream model structures;
joint-specific selection is handled by select_joint_biomechanics().

AngularVelocityVariant selects the modeled, preprocessed velocity surface:
  "original" : <joint>_angle_dot_stance (default)
  "norm"     : <joint>_angle_dot_stance_norm
The impedance package does not differentiate joint-angle trajectories.
%}

arguments
    rbfModelFile (1,1) string
    abReferenceFile (1,1) string
    options.AngularVelocityVariant (1,1) string {mustBeMember(options.AngularVelocityVariant,["original","norm"])} = "original"
end

if ~isfile(rbfModelFile)
    error('load_impedance_inputs:RBFFileNotFound', ...
        'RBF model file not found: %s', rbfModelFile);
end
if ~isfile(abReferenceFile)
    error('load_impedance_inputs:ReferenceFileNotFound', ...
        'AB reference file not found: %s', abReferenceFile);
end

rbfLoaded = load(rbfModelFile);
refLoaded = load(abReferenceFile);

if ~isfield(rbfLoaded, 'RBF_model_full')
    error('load_impedance_inputs:MissingRBFModel', ...
        'Expected variable "RBF_model_full" in %s.', rbfModelFile);
end
if ~isfield(refLoaded, 'target_biomechanics_total')
    error('load_impedance_inputs:MissingReference', ...
        'Expected variable "target_biomechanics_total" in %s.', abReferenceFile);
end

inputs = struct();
inputs.RBF_model = rbfLoaded.RBF_model_full;
inputs.AB_ref = refLoaded.target_biomechanics_total;
inputs.angular_velocity_variant = options.AngularVelocityVariant;

% Unified terrain-slope and phase domains used by impedance identification.
inputs.slope = linspace(-35, 35, 71);
inputs.stance_phase = linspace(0, 100, 100);
inputs.reference_slope = [ ...
    -35, -30, -24, -20, ...
    -11, -9, -7, -5, 0, 5, 7, 9, 11, ...
    20, 24, 30, 35];
inputs.reference_stance_phase = linspace(0, 100, 101);

inputs.source.rbf_model_file = rbfModelFile;
inputs.source.ab_reference_file = abReferenceFile;
inputs.source.angular_velocity_variant = options.AngularVelocityVariant;
end
