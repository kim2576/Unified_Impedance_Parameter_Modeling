function constraints = build_early_stance_constraints( ...
    phaseFraction, P, KUpper, BUpper, bounds, cfg)
%BUILD_EARLY_STANCE_CONSTRAINTS Construct ES equality, bound, and derivative constraints.

arguments
    phaseFraction (:,1) double {mustBeFinite, mustBeReal}
    P (:,:) double {mustBeFinite, mustBeReal}
    KUpper (1,1) double {mustBeFinite, mustBeReal}
    BUpper (1,1) double {mustBeFinite, mustBeReal}
    bounds (1,1) struct
    cfg (1,1) struct
end

order = cfg.polynomial.order;
nCoeff = cfg.polynomial.n_coefficients_per_parameter;

if order ~= 4 || nCoeff ~= 5
    error('build_early_stance_constraints:UnsupportedPolynomial', ...
        ['The stance parameterization uses fourth-order K and B polynomials. ' ...
         'Expected order=4 and five coefficients per parameter.']);
end
if size(P,1) ~= numel(phaseFraction) || size(P,2) ~= nCoeff
    error('build_early_stance_constraints:BasisSize', ...
        'Polynomial basis size is inconsistent with phase/configuration.');
end

% Heel-contact K/B lower-bound equality.
p0 = 0;
P0 = build_polynomial_basis(p0, order);
Aeq = [P0, zeros(1,nCoeff); zeros(1,nCoeff), P0];
beq = [bounds.K.lower; bounds.B.lower];

% Pointwise ES bounds.
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

% Derivative constraints around the ES reference phase.
peak = cfg.phase.ES_weight_peak / 100;
prePeak = phaseFraction(phaseFraction < peak);
postPeak = phaseFraction(phaseFraction >= peak);

% For q(p)=a4*p^4+...+a0, q'(p)=[4p^3 3p^2 2p 1 0]*a.
dPre = derivativeBasis(prePeak);
dPost = derivativeBasis(postPeak);
Zpre = zeros(size(dPre));
Zpost = zeros(size(dPost));

% K'(p) >= min  ->  -K'(p) <= -min
A_K_pre_min = [-dPre, Zpre];
b_K_pre_min = -cfg.ES.derivative.K.pre_peak_min * ones(size(dPre,1),1);

% post_peak_min <= K'(p) <= post_peak_max
A_K_post_max = [ dPost, Zpost];
b_K_post_max = cfg.ES.derivative.K.post_peak_max * ones(size(dPost,1),1);
A_K_post_min = [-dPost, Zpost];
b_K_post_min = -cfg.ES.derivative.K.post_peak_min * ones(size(dPost,1),1);

% B'(p) >= pre_peak_min
A_B_pre_min = [Zpre, -dPre];
b_B_pre_min = -cfg.ES.derivative.B.pre_peak_min * ones(size(dPre,1),1);

% post_peak_min <= B'(p) <= post_peak_max
A_B_post_max = [Zpost,  dPost];
b_B_post_max = cfg.ES.derivative.B.post_peak_max * ones(size(dPost,1),1);
A_B_post_min = [Zpost, -dPost];
b_B_post_min = -cfg.ES.derivative.B.post_peak_min * ones(size(dPost,1),1);

Aineq = [Aineq; ...
    A_K_pre_min; A_K_post_max; A_K_post_min; ...
    A_B_pre_min; A_B_post_max; A_B_post_min];
bineq = [bineq; ...
    b_K_pre_min; b_K_post_max; b_K_post_min; ...
    b_B_pre_min; b_B_post_max; b_B_post_min];

constraints = struct();
constraints.Aineq = Aineq;
constraints.bineq = bineq;
constraints.Aeq = Aeq;
constraints.beq = beq;
constraints.peak_phase_fraction = peak;
constraints.pre_peak_phase = prePeak;
constraints.post_peak_phase = postPeak;
end

function dP = derivativeBasis(p)
p = p(:);
dP = [4*p.^3, 3*p.^2, 2*p, ones(size(p)), zeros(size(p))];
end
