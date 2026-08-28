within Estimation.FusionHorizon;

function composePose
  "Carry a pose forward by one preintegral right factor"
  input Estimation.FusionHorizon.Pose pose;
  input Estimation.FusionHorizon.Delta delta;
  input Real gravityWorldEnu_m_s2[3];
  output Estimation.FusionHorizon.Pose advanced;
protected
  Real rotationWorldBody[3, 3];
  Real span_s;
algorithm
  // X(T) = L X(0) R with L = exp(M T) contributing the gravity blocks and R the
  // buffered (dp, dv, dq) triple: the FOH paper Theorem 6 (Sec. VI, exact
  // reapplication), equation 18, and the same composition order
  // Estimation.StrapdownINS.ESKF.predictPreintegrated uses for the nominal
  // state. Because L and R do not depend on the pose, a corrected horizon pose
  // reapplies the SAME buffered factors with no re-integration: that is what
  // makes the re-base an identity rather than a tuned tracking loop.
  span_s := delta.integrationTime_s;
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(pose.quaternionWorldBody);
  advanced := Estimation.FusionHorizon.Pose(
    positionWorldEnu_m=pose.positionWorldEnu_m
      + pose.velocityWorldEnu_m_s * span_s
      + 0.5 * gravityWorldEnu_m_s2 * span_s * span_s
      + rotationWorldBody * delta.deltaPositionBodyFlu_m,
    velocityWorldEnu_m_s=pose.velocityWorldEnu_m_s
      + gravityWorldEnu_m_s2 * span_s
      + rotationWorldBody * delta.deltaVelocityBodyFlu_m_s,
    quaternionWorldBody=LieGroups.SO3.Quat.normalize(
      LieGroups.SO3.Quat.product(
        pose.quaternionWorldBody, delta.deltaQuaternionBodyFlu)));
end composePose;
