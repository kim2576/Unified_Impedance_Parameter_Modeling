function equilibrium = build_equilibrium_angle(jointData, cfg)
%BUILD_EQUILIBRIUM_ANGLE Dispatch the joint-specific equilibrium-angle model.

arguments
    jointData (1,1) struct
    cfg (1,1) struct
end

if ~isfield(cfg, 'joint') || ~isfield(cfg, 'equilibrium') || ...
        ~isfield(cfg.equilibrium, 'strategy')
    error('build_equilibrium_angle:InvalidConfig', ...
        'cfg must contain cfg.joint and cfg.equilibrium.strategy.');
end
if ~isfield(jointData, 'joint') || ~isfield(jointData, 'angle') || ...
        ~isfield(jointData, 'moment') || ~isfield(jointData, 'slope')
    error('build_equilibrium_angle:InvalidJointData', ...
        'jointData must contain joint, angle, moment, and slope.');
end
if jointData.joint ~= cfg.joint
    error('build_equilibrium_angle:JointMismatch', ...
        'jointData.joint (%s) does not match cfg.joint (%s).', ...
        jointData.joint, cfg.joint);
end

switch cfg.equilibrium.strategy
    case "knee"
        equilibrium = build_knee_equilibrium(jointData, cfg);
    case "ankle"
        equilibrium = build_ankle_equilibrium(jointData, cfg);
    otherwise
        error('build_equilibrium_angle:UnknownStrategy', ...
            'Unsupported equilibrium strategy: %s', cfg.equilibrium.strategy);
end

if ~isequal(size(equilibrium.theta_eq), size(jointData.angle))
    error('build_equilibrium_angle:UnexpectedOutputSize', ...
        'theta_eq must match the joint angle surface size.');
end

equilibrium.angle_error = jointData.angle - equilibrium.theta_eq;
end
