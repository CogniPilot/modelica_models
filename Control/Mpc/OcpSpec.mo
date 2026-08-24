within Control.Mpc;

// SPDX-License-Identifier: Apache-2.0

record OcpSpec "Fixed-horizon quadratic tracking optimal control problem"
  Integer stateCount(min=1) = 2 "Plant state dimension nx";
  Integer inputCount(min=1) = 1 "Control input dimension nu";
  Integer horizonLength(min=1) = 10 "Number of shooting intervals N";
  Real samplePeriod(unit="s", min=1.0e-9) = 0.05
    "Control tick and shooting-interval length";
  Real regularization(min=0.0) = 1.0e-9
    "Diagonal damping the step solver may add to its normal matrix";
  annotation(Documentation(info="<html>
    <p>The tracking weight diagonals belong to this problem description as
    well, but the compiler currently rejects record components whose
    dimensions depend on sibling record members, so the weights live as
    parameters of <code>PartialTranscription</code>, sized there by
    <code>stateCount</code> and <code>inputCount</code>. Fold them back into
    this record once dependent record dimensions are supported.</p>
  </html>"));
end OcpSpec;
