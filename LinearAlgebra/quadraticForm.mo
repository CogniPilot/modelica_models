within LinearAlgebra;
function quadraticForm "Dimension-generic quadratic form x' P x"
  input Real x[:];
  input Real P[size(x, 1), size(x, 1)];
  output Real value;
algorithm
  value := x * (SymmetricMatrix.mulVec(SymmetricMatrix.packSymmetrized(P), x));
end quadraticForm;
