within Estimation.FusionHorizon;

function composeDelta
  "Compose two adjacent SE_2(3) preintegral right factors and their Jacobians"
  input Estimation.FusionHorizon.Delta first "Right factor over the earlier span";
  input Estimation.FusionHorizon.Delta second "Right factor over the later span";
  output Estimation.FusionHorizon.Delta composed;
protected
  Real firstRotation[3, 3] "R1, the rotation of the earlier factor";
  Real secondRotation[3, 3] "R2, the rotation of the later factor";
  Real secondSpan_s;
  Real rotatedPosition_m[3];
  Real rotatedVelocity_m_s[3];
  Real velocityCoupling[3, 3] "-R1 * wedge(dv2), the attitude-to-velocity term";
  Real positionCoupling[3, 3] "-R1 * wedge(dp2), the attitude-to-position term";
  Real composedPosition_m[3];
  Real composedVelocity_m_s[3];
  Real composedQuaternion[4];
algorithm
  // Adjacent right factors multiply, exactly: with X(T) = L X(0) R and L, R
  // independent of X(0) (FOH paper Lemma 3), two spans give
  // X = L(T1+T2) X(0) (R1 R2). Composing the buffer therefore reproduces a
  // single integration pass over the same samples, which is the composition
  // property of Lemma 5 (Sec. IV-C) and the reason a FIFO of deltas can stand
  // in for the raw sample history.
  //
  // The velocity-to-position coupling dv1 * dt2 is the nilpotent B block of the
  // extended algebra. It is what distinguishes this from the plain SE_2(3)
  // product in LieGroups.SE23.Quat.product, and dropping it is the classical
  // way to get an integrator that is right in attitude and wrong in position.
  firstRotation := LieGroups.SO3.Quat.to_DCM(first.deltaQuaternionBodyFlu);
  secondRotation := LieGroups.SO3.Quat.to_DCM(second.deltaQuaternionBodyFlu);
  secondSpan_s := second.integrationTime_s;
  rotatedPosition_m := firstRotation * second.deltaPositionBodyFlu_m;
  rotatedVelocity_m_s := firstRotation * second.deltaVelocityBodyFlu_m_s;

  composedPosition_m := first.deltaPositionBodyFlu_m
    + first.deltaVelocityBodyFlu_m_s * secondSpan_s
    + rotatedPosition_m;
  composedVelocity_m_s := first.deltaVelocityBodyFlu_m_s + rotatedVelocity_m_s;
  composedQuaternion := LieGroups.SO3.Quat.normalize(
    LieGroups.SO3.Quat.product(
      first.deltaQuaternionBodyFlu, second.deltaQuaternionBodyFlu));

  // Bias sensitivities are the first variation of the same composition, in the
  // right-perturbation convention R(b + db) = R(b) * Exp(J * db) that
  // preintegrateImuStep already produces. Conjugating the earlier rotation
  // Jacobian by R2 is what makes the chain associative:
  //   R1 Exp(J1 db) R2 Exp(J2 db) = (R1 R2) Exp((R2' J1 + J2) db).
  // The two coupling terms come from wedge(x) y = -wedge(y) x applied to the
  // rotated increments, and are formed once and reused so the composed
  // position and velocity Jacobians share them.
  velocityCoupling := -firstRotation
    * LieGroups.SO3.Quat.wedge(second.deltaVelocityBodyFlu_m_s);
  positionCoupling := -firstRotation
    * LieGroups.SO3.Quat.wedge(second.deltaPositionBodyFlu_m);

  composed := Estimation.FusionHorizon.Delta(
    deltaPositionBodyFlu_m=composedPosition_m,
    deltaVelocityBodyFlu_m_s=composedVelocity_m_s,
    deltaQuaternionBodyFlu=composedQuaternion,
    integrationTime_s=first.integrationTime_s + secondSpan_s,
    deltaRotationGyroscopeBiasJacobian_s=
      transpose(secondRotation) * first.deltaRotationGyroscopeBiasJacobian_s
        + second.deltaRotationGyroscopeBiasJacobian_s,
    deltaVelocityGyroscopeBiasJacobian_m=
      first.deltaVelocityGyroscopeBiasJacobian_m
        + velocityCoupling * first.deltaRotationGyroscopeBiasJacobian_s
        + firstRotation * second.deltaVelocityGyroscopeBiasJacobian_m,
    deltaVelocityAccelerometerBiasJacobian_s=
      first.deltaVelocityAccelerometerBiasJacobian_s
        + firstRotation * second.deltaVelocityAccelerometerBiasJacobian_s,
    deltaPositionGyroscopeBiasJacobian_m_s=
      first.deltaPositionGyroscopeBiasJacobian_m_s
        + first.deltaVelocityGyroscopeBiasJacobian_m * secondSpan_s
        + positionCoupling * first.deltaRotationGyroscopeBiasJacobian_s
        + firstRotation * second.deltaPositionGyroscopeBiasJacobian_m_s,
    deltaPositionAccelerometerBiasJacobian_s2=
      first.deltaPositionAccelerometerBiasJacobian_s2
        + first.deltaVelocityAccelerometerBiasJacobian_s * secondSpan_s
        + firstRotation * second.deltaPositionAccelerometerBiasJacobian_s2);
end composeDelta;
