within Estimation.FusionHorizon;

function navigationEstimate
  "Publish the predicted pose in the canonical estimator-independent fields"
  input Estimation.FusionHorizon.Pose pose;
  input Real angularVelocityBodyFlu_rad_s[3]
    "Already corrected by the estimator's gyroscope bias";
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real accelerometerBiasBodyFlu_m_s2[3];
  input Real gravityWorldEnu_m_s2[3];
  output Real positionWorldEnu_m[3];
  output Real velocityWorldEnu_m_s[3];
  output Real accelerationWorldEnu_m_s2[3];
  output Real quaternionWorldBody[4];
  output Real rotationWorldBody[3, 3];
  output Real eulerRpy_rad[3];
  output Real angularVelocityWorldEnu_rad_s[3];
protected
  Real eulerB321_rad[3];
  Real correctedSpecificForceBody_m_s2[3];
algorithm
  // Separate outputs rather than one Avionics.NavigationEstimate, and the
  // caller assigns the connector field by field. That is how every estimator in
  // this library publishes: a whole record assigned to a connector is not
  // something OpenModelica generates code for, and Rumoca counts it short of
  // the fields it defines, so the model comes out unbalanced.
  //
  // Same construction the filters publish, driven by the predicted pose at now
  // instead of the filter state at the horizon. The redundant attitude forms
  // are produced here rather than by the consumer so no consumer depends on a
  // representation choice.
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(pose.quaternionWorldBody);
  eulerB321_rad := LieGroups.SO3.EulerB321.from_Quat(pose.quaternionWorldBody);
  correctedSpecificForceBody_m_s2 := specificForceMeasuredBodyFlu_m_s2
    - accelerometerBiasBodyFlu_m_s2;
  positionWorldEnu_m := pose.positionWorldEnu_m;
  velocityWorldEnu_m_s := pose.velocityWorldEnu_m_s;
  accelerationWorldEnu_m_s2 := rotationWorldBody
    * correctedSpecificForceBody_m_s2 + gravityWorldEnu_m_s2;
  quaternionWorldBody := pose.quaternionWorldBody;
  eulerRpy_rad := {eulerB321_rad[3], eulerB321_rad[2], eulerB321_rad[1]};
  angularVelocityWorldEnu_rad_s := rotationWorldBody
    * angularVelocityBodyFlu_rad_s;
end navigationEstimate;
