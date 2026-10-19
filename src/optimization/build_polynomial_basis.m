function P = build_polynomial_basis(phaseFraction, order)
%{
Polynomial basis in descending power order.

P = build_polynomial_basis(phaseFraction, order)

For order = 4, this reproduces the legacy basis:
  [p.^4, p.^3, p.^2, p, 1]

Input phaseFraction may be a row or column vector. Output rows correspond
to phase samples; columns correspond to descending polynomial powers.
%}

arguments
    phaseFraction (:,1) double {mustBeFinite, mustBeReal}
    order (1,1) double {mustBeInteger, mustBeNonnegative}
end

powers = order:-1:0;
P = phaseFraction .^ powers;
end
