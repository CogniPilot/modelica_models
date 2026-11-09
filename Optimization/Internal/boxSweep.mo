within Optimization.Internal;
function boxSweep "One projected Gauss-Seidel sweep over all coordinates of a box QP"
  input Real H[:, size(H, 1)];
  input Real g[size(H, 1)];
  input Real lower[size(H, 1)];
  input Real upper[size(H, 1)];
  input Real xIn[size(H, 1)];
  output Real x[size(H, 1)];
protected
  Real gradient;
algorithm
  x := xIn;
  for i in 1:size(H, 1) loop
    gradient := g[i];
    for j in 1:size(H, 1) loop
      gradient := gradient + H[i, j] * x[j];
    end for;
    x[i] := min(max(x[i] - gradient / H[i, i], lower[i]), upper[i]);
  end for;
end boxSweep;
