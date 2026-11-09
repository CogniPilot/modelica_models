within Optimization.Internal;
function boxResidual "Largest coordinate step a projected Gauss-Seidel sweep would take"
  input Real H[:, size(H, 1)];
  input Real g[size(H, 1)];
  input Real lower[size(H, 1)];
  input Real upper[size(H, 1)];
  input Real x[size(H, 1)];
  output Real residual "In the units of x; zero exactly at the optimum";
protected
  Real gradient;
algorithm
  residual := 0.0;
  for i in 1:size(H, 1) loop
    gradient := g[i];
    for j in 1:size(H, 1) loop
      gradient := gradient + H[i, j] * x[j];
    end for;
    residual := max(residual,
      abs(x[i] - min(max(x[i] - gradient / H[i, i], lower[i]), upper[i])));
  end for;
end boxResidual;
