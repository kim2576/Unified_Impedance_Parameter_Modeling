function P = build_polynomial_basis(phaseFraction, order)
%BUILD_POLYNOMIAL_BASIS Polynomial basis in descending power order.

arguments
    phaseFraction (:,1) double {mustBeFinite, mustBeReal}
    order (1,1) double {mustBeInteger, mustBeNonnegative}
end

powers = order:-1:0;
P = phaseFraction .^ powers;
end
