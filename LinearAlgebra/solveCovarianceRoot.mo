within LinearAlgebra;

function solveCovarianceRoot "Solve L*L'*X=B without forming L*L'"
  input Real factor[:, size(factor, 1)];
  input Real rhs[size(factor, 1), :];
  output Real solution[size(rhs, 1), size(rhs, 2)];
  output Real whitened[size(rhs, 1), size(rhs, 2)];
  output Boolean ok;
protected
  Integer row;
  Real value;
algorithm
  (whitened, ok) := solveLower(factor, rhs);
  solution := zeros(size(rhs, 1), size(rhs, 2));
  for reverseRow in 1:size(factor, 1) loop
    row := size(factor, 1) + 1 - reverseRow;
    for column in 1:size(rhs, 2) loop
      value := whitened[row, column];
      for term in (row + 1):size(factor, 1) loop
        value := value - factor[term, row] * solution[term, column];
      end for;
      solution[row, column] := if ok then value / factor[row, row] else 0.0;
    end for;
  end for;
end solveCovarianceRoot;
