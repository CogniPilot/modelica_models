within LinearAlgebra;

function solveLower "Forward substitution through a lower triangular factor"
  input Real factor[:, size(factor, 1)];
  input Real rhs[size(factor, 1), :];
  output Real solution[size(rhs, 1), size(rhs, 2)];
  output Boolean ok;
protected
  Real value;
  Boolean pivotUsable;
algorithm
  solution := zeros(size(rhs, 1), size(rhs, 2));
  ok := true;
  for row in 1:size(factor, 1) loop
    pivotUsable := factor[row, row] > 0.0 and abs(factor[row, row]) < 1.0e30;
    ok := ok and pivotUsable;
    for column in 1:size(rhs, 2) loop
      value := rhs[row, column];
      for term in 1:(row - 1) loop
        value := value - factor[row, term] * solution[term, column];
      end for;
      solution[row, column] := if pivotUsable then value / factor[row, row] else 0.0;
    end for;
  end for;
end solveLower;
