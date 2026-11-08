within LinearAlgebra;

function solveLower "Forward substitution through a lower triangular factor"
  input Real factor[:, size(factor, 1)];
  input Real rhs[size(factor, 1), :];
  output Real solution[size(rhs, 1), size(rhs, 2)];
  output Boolean ok;
algorithm
  (solution, ok) := LowerTriangular.solve(LowerTriangular.pack(factor), rhs, true);
end solveLower;
