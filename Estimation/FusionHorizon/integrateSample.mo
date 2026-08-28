within Estimation.FusionHorizon;

function integrateSample
  "Integrate one IMU interval into a delta measured from the identity"
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real previousAngularVelocityMeasuredBodyFlu_rad_s[3]
    "Measured angular velocity at the interval start";
  input Real previousSpecificForceMeasuredBodyFlu_m_s2[3]
    "Measured specific force at the interval start";
  input Real gyroscopeBiasAnchorBodyFlu_rad_s[3];
  input Real accelerometerBiasAnchorBodyFlu_m_s2[3];
  input Real dt(unit = "s");
  input Boolean useFirstOrderHold;
  output Estimation.FusionHorizon.Delta delta;
protected
  Real deltaPosition_m[3];
  Real deltaVelocity_m_s[3];
  Real deltaQuaternion[4];
  Real rotationGyroscopeBiasJacobian_s[3, 3];
  Real velocityGyroscopeBiasJacobian_m[3, 3];
  Real velocityAccelerometerBiasJacobian_s[3, 3];
  Real positionGyroscopeBiasJacobian_m_s[3, 3];
  Real positionAccelerometerBiasJacobian_s2[3, 3];
algorithm
  // One closed-form update per IMU sample, started from the identity so the
  // result is a standalone right factor that composes with any neighbour.
  // Under the first-order hold the increment is the truncated Magnus exponent
  // of the FOH paper Theorem 1 (Sec. III-D): the trapezoid means plus one Lie
  // bracket, whose three components are the classical coning, sculling, and
  // scrolling corrections. The bracket has no time-block part, so the closed
  // form of the zero-order hold exponentiates it unchanged.
  (deltaPosition_m,
   deltaVelocity_m_s,
   deltaQuaternion,
   rotationGyroscopeBiasJacobian_s,
   velocityGyroscopeBiasJacobian_m,
   velocityAccelerometerBiasJacobian_s,
   positionGyroscopeBiasJacobian_m_s,
   positionAccelerometerBiasJacobian_s2) :=
    Estimation.StrapdownINS.preintegrateImuStep(
      zeros(3),
      zeros(3),
      {1.0, 0.0, 0.0, 0.0},
      zeros(3, 3),
      zeros(3, 3),
      zeros(3, 3),
      zeros(3, 3),
      zeros(3, 3),
      angularVelocityMeasuredBodyFlu_rad_s,
      specificForceMeasuredBodyFlu_m_s2,
      gyroscopeBiasAnchorBodyFlu_rad_s,
      accelerometerBiasAnchorBodyFlu_m_s2,
      dt,
      useFirstOrderHold,
      previousAngularVelocityMeasuredBodyFlu_rad_s,
      previousSpecificForceMeasuredBodyFlu_m_s2);
  delta := Estimation.FusionHorizon.Delta(
    deltaPositionBodyFlu_m=deltaPosition_m,
    deltaVelocityBodyFlu_m_s=deltaVelocity_m_s,
    deltaQuaternionBodyFlu=deltaQuaternion,
    integrationTime_s=dt,
    deltaRotationGyroscopeBiasJacobian_s=rotationGyroscopeBiasJacobian_s,
    deltaVelocityGyroscopeBiasJacobian_m=velocityGyroscopeBiasJacobian_m,
    deltaVelocityAccelerometerBiasJacobian_s=
      velocityAccelerometerBiasJacobian_s,
    deltaPositionGyroscopeBiasJacobian_m_s=positionGyroscopeBiasJacobian_m_s,
    deltaPositionAccelerometerBiasJacobian_s2=
      positionAccelerometerBiasJacobian_s2);
end integrateSample;
