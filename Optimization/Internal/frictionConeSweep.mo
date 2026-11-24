within Optimization.Internal;
function frictionConeSweep "One block projected Gauss-Seidel sweep over all contacts"
  input Real A[:, size(A, 1)];
  input Real b[size(A, 1)];
  input Real mu[:];
  input Real lipschitz[size(mu, 1)];
  input Real fIn[size(A, 1)];
  output Real f[size(A, 1)];
protected
  Real step[3];
algorithm
  f := fIn;
  for c in 1:size(mu, 1) loop
    step := Optimization.Internal.contactBlockStep(A, b, f, c, mu[c], lipschitz[c]);
    f[3 * c - 2] := step[1];
    f[3 * c - 1] := step[2];
    f[3 * c] := step[3];
  end for;
end frictionConeSweep;
