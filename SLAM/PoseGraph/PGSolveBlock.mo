within SLAM.PoseGraph;
function PGSolveBlock
  input Real L[6,6]; input Real b[6]; output Real x[6];
protected Real packedFactor[21]; Real z[6]; Real value; Integer row;
algorithm
  packedFactor := LinearAlgebra.LowerTriangular.pack(L);
  z := zeros(6); x := zeros(6); value := 0.0; row := 1;
  for i in 1:6 loop
    value := b[i];
    for k in 1:6 loop if k < i then value := value-packedFactor[LinearAlgebra.LowerTriangular.index(i, k)]*z[k]; end if; end for;
    z[i] := value/packedFactor[LinearAlgebra.LowerTriangular.index(i, i)];
  end for;
  for reverseRow in 1:6 loop
    row := 7-reverseRow; value := z[row];
    for k in 1:6 loop if k > row then value := value-packedFactor[LinearAlgebra.LowerTriangular.index(k, row)]*x[k]; end if; end for;
    x[row] := value/packedFactor[LinearAlgebra.LowerTriangular.index(row, row)];
  end for;
end PGSolveBlock;
