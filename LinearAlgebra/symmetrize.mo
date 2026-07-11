within LinearAlgebra;
function symmetrize "Average a square matrix with its transpose"
  input Real A[:, size(A, 1)] "Square matrix";
  output Real symmetricA[size(A, 1), size(A, 2)];
algorithm
  symmetricA := 0.5 * (A + transpose(A));
end symmetrize;
