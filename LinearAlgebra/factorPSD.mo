within LinearAlgebra;

function factorPSD "Lower factor of a positive semidefinite covariance"
  input Real covariance[:, size(covariance, 1)];
  output Real factor[size(covariance, 1), size(covariance, 1)];
  output Boolean ok;
protected
  Real packedFactor[div(size(covariance, 1) * (size(covariance, 1) + 1), 2)];
algorithm
  (packedFactor, ok) := SymmetricMatrix.factorPSD(size(covariance, 1),
    SymmetricMatrix.packSymmetrized(covariance));
  factor := LowerTriangular.unpack(size(covariance, 1), packedFactor);
end factorPSD;
