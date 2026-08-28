within Estimation.FusionHorizon;

function retireDelta
  "Divide the earliest right factor out of a composed window, exactly"
  input Estimation.FusionHorizon.Delta first
    "The factor being retired: the OLDEST window, still held in the ring";
  input Estimation.FusionHorizon.Delta composed
    "The maintained product first (x) second";
  output Estimation.FusionHorizon.Delta second
    "The product with `first` divided out of the left";
protected
  Real firstRotation[3, 3];
  Real secondRotation[3, 3];
  Real secondSpan_s;
  Real secondPosition_m[3];
  Real secondVelocity_m_s[3];
  Real secondQuaternion[4];
  Real velocityCoupling[3, 3];
  Real positionCoupling[3, 3];
algorithm
  // THE LEFT FACTOR DIVIDES OUT IN CLOSED FORM, and every block of it is
  // exact rather than first order. That is worth stating plainly because the
  // design record says otherwise: docs/delayed-fusion-horizon.md rejects the
  // peel partly on the grounds that "the peel's bias Jacobians are only first
  // order through the inverse, which is a silent error". For the composition
  // this package actually uses, they are not. The derivation is below and
  // Tests.HorizonPredictorTests measures the residual at floating point.
  //
  // The reason is structural. Read composeDelta as a map from `second` to
  // `composed` with `first` held fixed: every line is `something known` plus
  // `a known matrix times a block of second`, and the known matrix is always a
  // rotation. The map is affine with orthogonal leading coefficients, so
  // solving it is a subtraction and a transpose, never an inversion. The one
  // ordering constraint is that the couplings depend on second's own position
  // and velocity, so those two blocks must be recovered BEFORE the Jacobians
  // that use them, which is the order written here.
  //
  // Nothing here weakens the epoch contract the fold carries: a maintained
  // product and the pose it is composed onto must still name the same fusion
  // instant, so a retirement belongs on exactly the tick the ring drops that
  // window and not on the tick after it.
  firstRotation := LieGroups.SO3.Quat.to_DCM(first.deltaQuaternionBodyFlu);

  // ---- the group part -----------------------------------------------------
  // R1 R2 = R_W, so R2 = R1' R_W, and the same relation on the quaternion.
  secondQuaternion := LieGroups.SO3.Quat.normalize(
    LieGroups.SO3.Quat.product(
      LieGroups.SO3.Quat.inverse(first.deltaQuaternionBodyFlu),
      composed.deltaQuaternionBodyFlu));
  secondRotation := LieGroups.SO3.Quat.to_DCM(secondQuaternion);
  // The time block is additive, so it inverts by subtraction, and it has to be
  // recovered first because the position blocks below are affine in it.
  secondSpan_s := composed.integrationTime_s - first.integrationTime_s;
  // v_W = v1 + R1 v2
  secondVelocity_m_s := transpose(firstRotation)
    * (composed.deltaVelocityBodyFlu_m_s - first.deltaVelocityBodyFlu_m_s);
  // p_W = p1 + v1 T2 + R1 p2, with T2 already known.
  secondPosition_m := transpose(firstRotation)
    * (composed.deltaPositionBodyFlu_m - first.deltaPositionBodyFlu_m
      - first.deltaVelocityBodyFlu_m_s * secondSpan_s);

  // ---- the bias sensitivities --------------------------------------------
  // The couplings are the SAME expressions composeDelta forms, evaluated at
  // the second factor recovered above, which is why the two blocks have to be
  // recovered first.
  velocityCoupling := -firstRotation
    * LieGroups.SO3.Quat.wedge(secondVelocity_m_s);
  positionCoupling := -firstRotation
    * LieGroups.SO3.Quat.wedge(secondPosition_m);

  second := Estimation.FusionHorizon.Delta(
    deltaPositionBodyFlu_m=secondPosition_m,
    deltaVelocityBodyFlu_m_s=secondVelocity_m_s,
    deltaQuaternionBodyFlu=secondQuaternion,
    integrationTime_s=secondSpan_s,
    // Jr_W = R2' Jr1 + Jr2
    deltaRotationGyroscopeBiasJacobian_s=
      composed.deltaRotationGyroscopeBiasJacobian_s
        - transpose(secondRotation) * first.deltaRotationGyroscopeBiasJacobian_s,
    // Jvg_W = Jvg1 + Cv Jr1 + R1 Jvg2
    deltaVelocityGyroscopeBiasJacobian_m=transpose(firstRotation)
      * (composed.deltaVelocityGyroscopeBiasJacobian_m
        - first.deltaVelocityGyroscopeBiasJacobian_m
        - velocityCoupling * first.deltaRotationGyroscopeBiasJacobian_s),
    // Jva_W = Jva1 + R1 Jva2
    deltaVelocityAccelerometerBiasJacobian_s=transpose(firstRotation)
      * (composed.deltaVelocityAccelerometerBiasJacobian_s
        - first.deltaVelocityAccelerometerBiasJacobian_s),
    // Jpg_W = Jpg1 + Jvg1 T2 + Cp Jr1 + R1 Jpg2
    deltaPositionGyroscopeBiasJacobian_m_s=transpose(firstRotation)
      * (composed.deltaPositionGyroscopeBiasJacobian_m_s
        - first.deltaPositionGyroscopeBiasJacobian_m_s
        - first.deltaVelocityGyroscopeBiasJacobian_m * secondSpan_s
        - positionCoupling * first.deltaRotationGyroscopeBiasJacobian_s),
    // Jpa_W = Jpa1 + Jva1 T2 + R1 Jpa2
    deltaPositionAccelerometerBiasJacobian_s2=transpose(firstRotation)
      * (composed.deltaPositionAccelerometerBiasJacobian_s2
        - first.deltaPositionAccelerometerBiasJacobian_s2
        - first.deltaVelocityAccelerometerBiasJacobian_s * secondSpan_s));
end retireDelta;
