within LinearAlgebra;
function symmetrize "Average a square matrix with its transpose"
  input Real A[:, size(A, 1)] "Square matrix";
  output Real symmetricA[size(A, 1), size(A, 2)];
algorithm
  symmetricA := SymmetricMatrix.unpack(size(A, 1), SymmetricMatrix.packSymmetrized(A));
end symmetrize;
