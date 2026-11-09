within Optimization.Internal;
function contactBlockStep "One projected gradient step on one contact block"
  input Real A[:, size(A, 1)];
  input Real b[size(A, 1)];
  input Real f[size(A, 1)];
  input Integer contact "One-based contact index";
  input Real mu "Friction coefficient of this contact";
  input Real lipschitz "Positive step scale for this contact block";
  output Real step[3] "Candidate block value after the projected step";
protected
  Real gradient[3];
  Integer row;
  Real normal;
  Real tangent1;
  Real tangent2;
algorithm
  for k in 1:3 loop
    row := 3 * (contact - 1) + k;
    gradient[k] := b[row];
    for j in 1:size(A, 1) loop
      gradient[k] := gradient[k] + A[row, j] * f[j];
    end for;
  end for;
  (normal, tangent1, tangent2) := Optimization.Internal.projectCone(
    f[3 * contact - 2] - gradient[1] / lipschitz,
    f[3 * contact - 1] - gradient[2] / lipschitz,
    f[3 * contact] - gradient[3] / lipschitz,
    mu);
  step := {normal, tangent1, tangent2};
end contactBlockStep;
