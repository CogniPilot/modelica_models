within Tests.Assertions;
function spdAccepted "Return one when an SPD solve is accepted, zero otherwise"
  input Real A[:, size(A, 1)];
  output Real accepted;
protected
  Real X[size(A, 1), 1];
  Boolean ok;
algorithm
  (X, ok) := LinearAlgebra.solveSPD(A, ones(size(A, 1), 1));
  accepted := if ok then 1.0 else 0.0;
end spdAccepted;
