within Tests.RuleChecks;
function retrodictChain
  "Tangent map of retrodict built by chaining LieGroups derivative rules"
  input Estimation.StrapdownINS.ESKF.State current;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real gravityWorldEnu_m_s2[3];
  input Real age_s(unit = "s");
  output Real chain[15, 15]
    "d(delayed tangent)/d(current tangent), ordered position, velocity, attitude, gyro bias, accelerometer bias";
protected
  Real correctedAngularVelocity[3];
  Real correctedSpecificForce[3];
  Real currentExtendedPose[10];
  Real leftIncrement[9];
  Real rightIncrement[9];
  Real coupling[2, 2];
  Real stateFactor[9, 9];
  Real leftIncrementFactor[9, 9];
  Real incrementFromBias[9, 6];
algorithm
  // Same construction retrodict performs, so the chain is differentiating the
  // function the estimator actually calls.
  correctedAngularVelocity := angularVelocityMeasuredBodyFlu_rad_s
    - current.gyroscopeBiasBodyFlu_rad_s;
  correctedSpecificForce := specificForceMeasuredBodyFlu_m_s2
    - current.accelerometerBiasBodyFlu_m_s2;
  currentExtendedPose := cat(1, current.positionWorldEnu_m,
    current.velocityWorldEnu_m_s, current.quaternionWorldBody);
  leftIncrement := cat(1, zeros(3), -correctedSpecificForce * age_s,
    -correctedAngularVelocity * age_s);
  rightIncrement := cat(1, zeros(3), -gravityWorldEnu_m_s2 * age_s,
    zeros(3));
  coupling := [0.0, -age_s; 0.0, 0.0];

  stateFactor := LieGroups.SE23.Quat.exp_mixed_state_jacobian(
    currentExtendedPose, leftIncrement, rightIncrement, coupling);
  leftIncrementFactor := LieGroups.SE23.Quat.exp_mixed_left_increment_jacobian(
    currentExtendedPose, leftIncrement, rightIncrement, coupling);

  // The biases reach the delayed pose only by shifting the left increment:
  // the rotation slot picks up +age from the gyroscope bias and the
  // acceleration slot +age from the accelerometer bias.
  incrementFromBias := zeros(9, 6);
  incrementFromBias[7:9, 1:3] := age_s * identity(3);
  incrementFromBias[4:6, 4:6] := age_s * identity(3);

  chain := zeros(15, 15);
  chain[1:9, 1:9] := stateFactor;
  chain[1:9, 10:15] := leftIncrementFactor * incrementFromBias;
  chain[10:15, 10:15] := identity(6);
  annotation(Documentation(info="<html>
    <p>retrodict is one call to <code>exp_mixed</code> wrapped in an affine
    construction of its increments, so its tangent map factors exactly:
    the pose block is the exp_mixed state rule, the bias block is the exp_mixed
    left-increment rule composed with the increment's own derivative in the
    biases, and the bias states pass through unchanged.</p>
  </html>"));
end retrodictChain;
