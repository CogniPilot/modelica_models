within LinearAlgebra;
function contractionResidual
  "Matrix residual M_dot + A' M + M A + 2 rate M for contraction certificates"
  input Real A[:, :];
  input Real M[size(A, 1), size(A, 1)];
  input Real MDerivative[size(A, 1), size(A, 1)] =
    zeros(size(A, 1), size(A, 1));
  input Real rate(min=0.0) = 0.0 "Requested exponential contraction rate";
  output Real residual[size(A, 1), size(A, 1)];
algorithm
  assert(size(A, 1) == size(A, 2),
    "The contraction Jacobian must be square");
  residual := LinearAlgebra.symmetrize(
    MDerivative + transpose(A) * M + M * A + 2.0 * rate * M);
end contractionResidual;
