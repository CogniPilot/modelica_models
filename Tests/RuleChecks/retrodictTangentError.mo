within Tests.RuleChecks;
function retrodictTangentError
  "Delayed-tangent error retrodict produces for one perturbation of the current state"
  input Estimation.StrapdownINS.ESKF.State current;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real gravityWorldEnu_m_s2[3];
  input Real age_s(unit = "s");
  input Real perturbation[15]
    "Current-tangent perturbation: pose on the right, biases additive";
  input Real baseInverse[10] "Inverse of the unperturbed delayed extended pose";
  input Real baseBiases[6] "Unperturbed delayed gyroscope and accelerometer biases";
  output Real tangentError[15];
protected
  Real perturbedPose[10];
  Real perturbedVector[16];
algorithm
  perturbedPose := LieGroups.SE23.Quat.product(
    cat(1, current.positionWorldEnu_m, current.velocityWorldEnu_m_s,
      current.quaternionWorldBody),
    LieGroups.SE23.Quat.exp_map(perturbation[1:9]));
  perturbedVector := Estimation.StrapdownINS.ESKF.retrodict(
    Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m = perturbedPose[1:3],
      velocityWorldEnu_m_s = perturbedPose[4:6],
      quaternionWorldBody = perturbedPose[7:10],
      gyroscopeBiasBodyFlu_rad_s = current.gyroscopeBiasBodyFlu_rad_s
        + perturbation[10:12],
      accelerometerBiasBodyFlu_m_s2 = current.accelerometerBiasBodyFlu_m_s2
        + perturbation[13:15],
      covariance = zeros(15, 15)),
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2, age_s);
  tangentError := cat(1,
    LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.product(
      baseInverse, perturbedVector[1:10])),
    perturbedVector[11:16] - baseBiases);
end retrodictTangentError;
