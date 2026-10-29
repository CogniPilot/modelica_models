within LinearAlgebra;

function solveCovarianceRoot "Solve L*L'*X=B without forming L*L'"
  input Real factor[:, size(factor, 1)];
  input Real rhs[size(factor, 1), :];
  output Real solution[size(rhs, 1), size(rhs, 2)];
  output Real whitened[size(rhs, 1), size(rhs, 2)];
  output Boolean ok;
protected
  Real packedFactor[div(size(factor, 1) * (size(factor, 1) + 1), 2)];
  Boolean backwardOk;
algorithm
  packedFactor := LowerTriangular.pack(factor);
  (whitened, ok) := LowerTriangular.solve(packedFactor, rhs, true);
  solution := zeros(size(rhs, 1), size(rhs, 2));
  if ok then
    (solution, backwardOk) := UpperTriangular.solve(
      LowerTriangular.transposePacked(size(factor, 1), packedFactor), whitened, true);
    ok := ok and backwardOk;
  end if;
end solveCovarianceRoot;
