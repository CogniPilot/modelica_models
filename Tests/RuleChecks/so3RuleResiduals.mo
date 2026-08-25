within Tests.RuleChecks;
function so3RuleResiduals
  "Worst |rule - central difference| over randomized SO(3) points"
  input Real angleScale "Magnitude of the rotation vector each rule is evaluated at";
  input Real step "Central-difference step";
  input Integer trials "Number of randomized points";
  output Real worst[7]
    "exp_map, log_map, product left, product right, inverse, rotate rotation, rotate vector";
protected
  Real draw[12];
  Real axis[3];
  Real axisNorm;
  Real v[3];
  Real q[4];
  Real p[4];
  Real target[3];
  // Scalar accumulators rather than accumulation into the output array:
  // an array element assigned inside the trial loop makes the whole array a
  // fresh value each iteration, and evaluators that do not share that value
  // pay for one copy per element per iteration.
  Real worstExponential;
  Real worstLogarithm;
  Real worstProductLeft;
  Real worstProductRight;
  Real worstInverse;
  Real worstRotateRotation;
  Real worstRotateVector;
algorithm
  worstExponential := 0.0;
  worstLogarithm := 0.0;
  worstProductLeft := 0.0;
  worstProductRight := 0.0;
  worstInverse := 0.0;
  worstRotateRotation := 0.0;
  worstRotateVector := 0.0;
  for trial in 1:trials loop
    draw := Tests.RuleChecks.pseudoRandom(7919 * trial + 13, 12);
    axis := draw[1:3];
    axisNorm := sqrt(axis[1]^2 + axis[2]^2 + axis[3]^2);
    v := (angleScale / max(axisNorm, 1.0e-12)) * axis;
    q := LieGroups.SO3.Quat.exp_map(v);
    p := LieGroups.SO3.Quat.exp_map(2.0 * draw[4:6]);
    target := 5.0 * draw[7:9];

    worstExponential := max(worstExponential, Tests.Assertions.maxAbsMatrix(
      LieGroups.SO3.Quat.exp_map_jacobian(v)
      - Tests.RuleChecks.fdSo3ExpMapJacobian(v, step)));
    worstLogarithm := max(worstLogarithm, Tests.Assertions.maxAbsMatrix(
      LieGroups.SO3.Quat.log_map_jacobian(q)
      - Tests.RuleChecks.fdSo3LogMapJacobian(q, step)));
    worstProductLeft := max(worstProductLeft, Tests.Assertions.maxAbsMatrix(
      LieGroups.SO3.Quat.product_left_factor_jacobian(q, p)
      - Tests.RuleChecks.fdSo3ProductJacobian(q, p, 1, step)));
    worstProductRight := max(worstProductRight, Tests.Assertions.maxAbsMatrix(
      LieGroups.SO3.Quat.product_right_factor_jacobian(q, p)
      - Tests.RuleChecks.fdSo3ProductJacobian(q, p, 2, step)));
    worstInverse := max(worstInverse, Tests.Assertions.maxAbsMatrix(
      LieGroups.SO3.Quat.inverse_jacobian(q)
      - Tests.RuleChecks.fdSo3InverseJacobian(q, step)));
    worstRotateRotation := max(worstRotateRotation, Tests.Assertions.maxAbsMatrix(
      LieGroups.SO3.Quat.rotate_rotation_jacobian(q, target)
      - Tests.RuleChecks.fdSo3RotateJacobian(q, target, 1, step)));
    worstRotateVector := max(worstRotateVector, Tests.Assertions.maxAbsMatrix(
      LieGroups.SO3.Quat.rotate_vector_jacobian(q, target)
      - Tests.RuleChecks.fdSo3RotateJacobian(q, target, 2, step)));
  end for;
  worst := {worstExponential, worstLogarithm, worstProductLeft,
    worstProductRight, worstInverse, worstRotateRotation, worstRotateVector};
end so3RuleResiduals;
