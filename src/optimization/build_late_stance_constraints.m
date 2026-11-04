function constraints = build_late_stance_constraints( ...
    phaseFraction, P, earlyEndK, earlyEndB, slope, bounds, cfg)
%{
Construct late-stance constraints.

Unknown vector:
  x = [K polynomial coefficients; B polynomial coefficients]

The terminal early-stance K and B values initialize late stance. Equality
constraints preserve support-level impedance through the configured stiffness
reference phase p_K and return K and B toward their minimum values at toe-off.
The LS1 stiffness-derivative constraint can be applied over the complete
LS-start-to-p_K interval ("interval") or at p_K only ("peak_only").
%}

arguments
    phaseFraction (:,1) double {mustBeFinite, mustBeReal}
    P (:,:) double {mustBeFinite, mustBeReal}
    earlyEndK (1,1) double {mustBeFinite, mustBeReal}
    earlyEndB (1,1) double {mustBeFinite, mustBeReal}
    slope (1,1) double {mustBeFinite, mustBeReal}
    bounds (1,1) struct
    cfg (1,1) struct
end

order = cfg.polynomial.order;
nCoeff = cfg.polynomial.n_coefficients_per_parameter;

if order ~= 4 || nCoeff ~= 5
    error('build_late_stance_constraints:UnsupportedPolynomial', ...
        ['The stance parameterization uses fourth-order K and B polynomials. ' ...
         'Expected order=4 and five coefficients per parameter.']);
end
if size(P,1) ~= numel(phaseFraction) || size(P,2) ~= nCoeff
    error('build_late_stance_constraints:BasisSize', ...
        'Polynomial basis size is inconsistent with phase/configuration.');
end

% Locate the LS stiffness-reference phase p_K. Because phaseFraction begins
% at LS_start, its relative index is scale_duration + 1.
peakRelativeIndex = cfg.LS.scale_duration + 1;
if peakRelativeIndex ~= round(peakRelativeIndex) || ...
        peakRelativeIndex < 1 || peakRelativeIndex > numel(phaseFraction)
    error('build_late_stance_constraints:PeakIndex', ...
        'Configured LS peak index is outside the late-stance phase domain.');
end
peakRelativeIndex = round(peakRelativeIndex);
peakPhase = phaseFraction(peakRelativeIndex);

% Equality constraints at LS start and stance end:
% K(LS start) = K_ES(end), K(100%%) = K_lower
% B(LS start) = B_ES(end), B(100%%) = B_lower
PStart = build_polynomial_basis(phaseFraction(1), order);
PEnd = build_polynomial_basis(1, order);
AeqK = [PStart, zeros(1,nCoeff); PEnd, zeros(1,nCoeff)];
AeqB = [zeros(1,nCoeff), PStart; zeros(1,nCoeff), PEnd];
Aeq = [AeqK; AeqB];
beq = [earlyEndK; bounds.K.lower; earlyEndB; bounds.B.lower];

% Equality at configured LS K peak.
PPeak = build_polynomial_basis(peakPhase, order);
AeqPeak = [PPeak, zeros(1,nCoeff)];

switch cfg.LS.peak_scale_mode
    case "all"
        peakKTarget = cfg.LS.K_scale * earlyEndK;
    case "descent_only"
        if slope < cfg.equilibrium.descent_cutoff + cfg.LS.peak_scale_cutoff_offset
            peakKTarget = cfg.LS.K_scale * earlyEndK;
        else
            peakKTarget = earlyEndK;
        end
    otherwise
        error('build_late_stance_constraints:PeakScaleMode', ...
            'Unknown cfg.LS.peak_scale_mode: %s', cfg.LS.peak_scale_mode);
end

Aeq = [Aeq; AeqPeak];
beq = [beq; peakKTarget];

% Pointwise LS bounds. The terminal ES values define the support-level upper
% limits carried into LS; stiffness may additionally be scaled by K_scale.
KUpper = cfg.LS.K_scale * earlyEndK;
BUpper = earlyEndB;

A_K_upper = [ P, zeros(size(P))];
b_K_upper = KUpper * ones(size(P,1),1);
A_K_lower = [-P, zeros(size(P))];
b_K_lower = -bounds.K.lower * ones(size(P,1),1);

A_B_upper = [zeros(size(P)),  P];
b_B_upper = BUpper * ones(size(P,1),1);
A_B_lower = [zeros(size(P)), -P];
b_B_lower = -bounds.B.lower * ones(size(P,1),1);

Aineq = [A_K_upper; A_K_lower; A_B_upper; A_B_lower];
bineq = [b_K_upper; b_K_lower; b_B_upper; b_B_lower];

% LS1 stiffness derivative constraint:
%   K'(p) >= cfg.LS.derivative.K.pre_peak_min
% "interval" applies the lower derivative bound throughout LS1 (LS start to p_K).
% "peak_only" applies the same bound only at p_K. LS2 derivative limits below
% regulate constant-or-decreasing impedance during unloading.
prePeakMode = string(cfg.LS.derivative.K.pre_peak_mode);
switch prePeakMode
    case "interval"
        prePeak = phaseFraction(phaseFraction <= peakPhase);
    case "peak_only"
        prePeak = peakPhase;
    otherwise
        error('build_late_stance_constraints:PrePeakMode', ...
            'Unknown cfg.LS.derivative.K.pre_peak_mode: %s', prePeakMode);
end
DPre = derivativeBasis(prePeak);
ZPre = zeros(size(DPre));
A_K_pre_min = [-DPre, ZPre];
b_K_pre_min = -cfg.LS.derivative.K.pre_peak_min * ones(size(DPre,1),1);

% From p_K to toe-off, upper derivative limits regulate unloading behavior.
postPeak = phaseFraction(phaseFraction >= peakPhase);
DPost = derivativeBasis(postPeak);
ZPost = zeros(size(DPost));

A_K_post_max = [DPost, ZPost];
b_K_post_max = cfg.LS.derivative.K.post_peak_max * ones(size(DPost,1),1);

A_B_post_max = [ZPost, DPost];
b_B_post_max = cfg.LS.derivative.B.post_peak_max * ones(size(DPost,1),1);

Aineq = [Aineq; A_K_pre_min; A_K_post_max; A_B_post_max];
bineq = [bineq; b_K_pre_min; b_K_post_max; b_B_post_max];

constraints = struct();
constraints.Aineq = Aineq;
constraints.bineq = bineq;
constraints.Aeq = Aeq;
constraints.beq = beq;
constraints.peak_relative_index = peakRelativeIndex;
constraints.peak_phase_fraction = peakPhase;
constraints.peak_K_target = peakKTarget;
constraints.K_upper = KUpper;
constraints.B_upper = BUpper;
constraints.pre_peak_phase = prePeak;
constraints.pre_peak_mode = prePeakMode;
constraints.pre_peak_K_min = cfg.LS.derivative.K.pre_peak_min;
constraints.pre_peak_row_count = size(DPre,1);
constraints.post_peak_phase = postPeak;
end

function dP = derivativeBasis(p)
p = p(:);
dP = [4*p.^3, 3*p.^2, 2*p, ones(size(p)), zeros(size(p))];
end
