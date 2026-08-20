within Estimation.StrapdownINS.UKF;

function predictPreintegratedNominalVector
  "Apply a shared IMU preintegral to one UKF sigma-state bias hypothesis"
  input Estimation.StrapdownINS.UKF.NominalVector previous;
  input Avionics.ImuSample imu;
  input Real gravityWorldEnu_m_s2[3];
  output Estimation.StrapdownINS.UKF.NominalVector predicted;
protected
  Real deltaPositionBodyFlu_m[3];
  Real deltaVelocityBodyFlu_m_s[3];
  Real deltaQuaternionBodyFlu[4];
  Real previousRotationWorldBody[3, 3];
  Real dt;
algorithm
  dt := imu.integrationTime_s;
  (deltaPositionBodyFlu_m,
   deltaVelocityBodyFlu_m_s,
   deltaQuaternionBodyFlu) :=
    Estimation.StrapdownINS.correctPreintegratedImu(
      imu, previous[11:13], previous[14:16]);
  previousRotationWorldBody :=
    LieGroups.SO3.Quat.to_DCM(previous[7:10]);
  predicted := cat(1,
    previous[1:3] + previous[4:6] * dt
      + 0.5 * gravityWorldEnu_m_s2 * dt * dt
      + previousRotationWorldBody * deltaPositionBodyFlu_m,
    previous[4:6] + gravityWorldEnu_m_s2 * dt
      + previousRotationWorldBody * deltaVelocityBodyFlu_m_s,
    LieGroups.SO3.Quat.normalize(
      LieGroups.SO3.Quat.product(
        previous[7:10], deltaQuaternionBodyFlu)),
    previous[11:16]);
end predictPreintegratedNominalVector;
