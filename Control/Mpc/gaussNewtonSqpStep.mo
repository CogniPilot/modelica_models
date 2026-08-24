within Control.Mpc;

// SPDX-License-Identifier: Apache-2.0

function gaussNewtonSqpStep
  "Damped Gauss-Newton step from the dense normal equations via Cholesky"
  extends PartialNlpSolver;
protected
  Real normalMatrix[size(jacobian, 2), size(jacobian, 2)];
  Real negativeGradient[size(jacobian, 2), 1];
  Real solution[size(jacobian, 2), 1];
  Real accumulator;
  Boolean choleskyOk;
algorithm
  normalMatrix := transpose(jacobian) * jacobian;
  for i in 1:size(jacobian, 2) loop
    normalMatrix[i, i] := normalMatrix[i, i] + regularization;
    accumulator := 0.0;
    for row in 1:size(jacobian, 1) loop
      accumulator := accumulator - jacobian[row, i] * residual[row];
    end for;
    negativeGradient[i, 1] := accumulator;
  end for;
  (solution, choleskyOk) := LinearAlgebra.solveSPD(
    normalMatrix, negativeGradient);
  for i in 1:size(jacobian, 2) loop
    step[i] := solution[i, 1];
  end for;
  accepted := choleskyOk;
  annotation(Documentation(info="<html>
    <p>Solves <code>(J'J + regularization*I) step = -J'residual</code> with
    <code>LinearAlgebra.solveSPD</code>. For a residual assembled from a
    quadratic tracking cost this is the Gauss-Newton SQP step; it needs only
    first-order information. When the Cholesky factorization rejects the
    normal matrix the step must not be applied and <code>accepted</code> is
    false.</p>
  </html>"));
end gaussNewtonSqpStep;
