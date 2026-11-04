function jointData = select_joint_biomechanics(inputs, joint)
%{
Standardize knee/ankle RBF inputs for impedance fitting.

Output orientation used by the impedance optimizer:
  rows    = terrain slopes (71)
  columns = normalized stance-phase samples (100)

jointData contains:
  angle             [71 x 100] deg
  angular_velocity  [71 x 100] deg/s (selected preprocessed RBF field)
  moment            [71 x 100] Nm/kg
  swing_angle       [71 x 100] deg
  reference_moment  [17 x 101] Nm/kg
  angular_velocity_field  Source RBF field name
%}

arguments
    inputs (1,1) struct
    joint (1,1) string {mustBeMember(joint,["knee","ankle"])}
end

requiredInputFields = {'RBF_model','AB_ref','slope','stance_phase', ...
    'reference_slope','reference_stance_phase','angular_velocity_variant'};
for i = 1:numel(requiredInputFields)
    if ~isfield(inputs, requiredInputFields{i})
        error('select_joint_biomechanics:InvalidInputs', ...
            'Missing inputs.%s.', requiredInputFields{i});
    end
end

jointChar = char(joint);
angleField = [jointChar '_angle_stance'];
momentField = [jointChar '_moment_stance'];
switch inputs.angular_velocity_variant
    case "original"
        velocityField = [jointChar '_angle_dot_stance'];
    case "norm"
        velocityField = [jointChar '_angle_dot_stance_norm'];
    otherwise
        error('select_joint_biomechanics:UnknownVelocityVariant', ...
            'Unsupported angular velocity variant: %s.', ...
            inputs.angular_velocity_variant);
end
refMomentField = ['avg_' jointChar '_moment_stance_total'];

assertRBFField(inputs.RBF_model, angleField);
assertRBFField(inputs.RBF_model, momentField);
assertRBFField(inputs.RBF_model, velocityField);
if ~isfield(inputs.AB_ref, refMomentField)
    error('select_joint_biomechanics:MissingReferenceField', ...
        'Missing AB reference field: %s', refMomentField);
end

% Use the first 100 stance samples and transpose to slope-by-phase form.
angle = inputs.RBF_model.(angleField).RBF_in_full_domain(1:100, :)';
moment = inputs.RBF_model.(momentField).RBF_in_full_domain(1:100, :)';

% Angular velocity is an independently modeled RBF surface, constructed
% upstream from the preprocessed velocity measurements (or '_norm' inputs).
% Keep the established first-100-sample selection used by angle and moment.
% Do NOT recompute it by differentiating the fitted joint-angle surface.
angularVelocity = inputs.RBF_model.(velocityField).RBF_in_full_domain(1:100, :)';

jointData = struct();
jointData.joint = joint;
jointData.angle = angle;
jointData.angular_velocity = angularVelocity;
jointData.angular_velocity_field = string(velocityField);
jointData.angular_velocity_variant = inputs.angular_velocity_variant;
jointData.moment = moment;
jointData.swing_angle = [];

if joint == "ankle"
    swingField = 'ankle_angle_swing';
    assertRBFField(inputs.RBF_model, swingField);
    jointData.swing_angle = ...
        inputs.RBF_model.(swingField).RBF_in_full_domain(1:100, :)';
end

refMoment = inputs.AB_ref.(refMomentField);
if isequal(size(refMoment), [101, 17])
    refMoment = refMoment';
elseif ~isequal(size(refMoment), [17, 101])
    error('select_joint_biomechanics:UnexpectedReferenceSize', ...
        ['Expected %s to be 101x17 or 17x101; got %dx%d.'], ...
        refMomentField, size(refMoment,1), size(refMoment,2));
end
jointData.reference_moment = refMoment;

jointData.slope = inputs.slope;
jointData.phase = inputs.stance_phase;
jointData.reference_slope = inputs.reference_slope;
jointData.reference_phase = inputs.reference_stance_phase;

validateSurfaceSize(jointData.angle, numel(inputs.slope), 100, 'angle');
validateSurfaceSize(jointData.angular_velocity, numel(inputs.slope), 100, 'angular_velocity');
validateSurfaceSize(jointData.moment, numel(inputs.slope), 100, 'moment');
if joint == "ankle"
    validateSurfaceSize(jointData.swing_angle, numel(inputs.slope), 100, 'swing_angle');
end
end

function assertRBFField(rbfModel, fieldName)
if ~isfield(rbfModel, fieldName)
    error('select_joint_biomechanics:MissingRBFField', ...
        'Missing RBF field: %s', fieldName);
end
if ~isfield(rbfModel.(fieldName), 'RBF_in_full_domain')
    error('select_joint_biomechanics:MissingSurface', ...
        'Missing %s.RBF_in_full_domain.', fieldName);
end
end

function validateSurfaceSize(surface, nSlope, nPhase, label)
if ~isequal(size(surface), [nSlope, nPhase])
    error('select_joint_biomechanics:UnexpectedSurfaceSize', ...
        'Expected %s surface to be %dx%d; got %dx%d.', ...
        label, nSlope, nPhase, size(surface,1), size(surface,2));
end
end
