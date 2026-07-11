within LinearAlgebra;
function quadraticFormDerivative
  "Time derivative of x' P x for supplied x_dot and P_dot"
  input Real x[:];
  input Real xDerivative[size(x, 1)];
  input Real P[size(x, 1), size(x, 1)];
  input Real PDerivative[size(x, 1), size(x, 1)] = zeros(size(x, 1), size(x, 1));
  output Real valueDerivative;
protected
  Real symmetricP[size(x, 1), size(x, 1)];
algorithm
  symmetricP := LinearAlgebra.symmetrize(P);
  valueDerivative := 2.0 * xDerivative * (symmetricP * x)
    + x * (LinearAlgebra.symmetrize(PDerivative) * x);
end quadraticFormDerivative;
