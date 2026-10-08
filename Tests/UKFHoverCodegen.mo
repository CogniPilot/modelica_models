within Tests;
block UKFHoverCodegen
  "Native codegen regression: symmetric sigma points must preserve hover"
  input Boolean first;
  input Boolean preintegrated;
  output Real position[3](each start=0, each fixed=true);
  output Real velocity[3](each start=0, each fixed=true);
  output Real quaternion[4](start={1.0,0.0,0.0,0.0}, each fixed=true);
  output Real gyroBias[3](each start=0, each fixed=true);
  output Real accelBias[3](each start=0, each fixed=true);
  output Real covariance[15,15](each start=0, each fixed=true);
  output Boolean success(start=false, fixed=true);

 function run
   input Boolean first;
   input Boolean preintegrated;
   input Estimation.StrapdownINS.UKF.State previous;
   output Real position[3];
   output Real velocity[3];
   output Real quaternion[4];
   output Real gyroBias[3];
   output Real accelBias[3];
   output Real covariance[15,15];
   output Boolean success;
 protected
   Estimation.StrapdownINS.UKF.State state;
  Avionics.ImuSample imu;
algorithm
    imu := Avionics.ImuSample(valid=true, fresh=true, timestamp_s=0.0,
      angularVelocityBodyFlu_rad_s=zeros(3), specificForceBodyFlu_m_s2={0.0,0.0,9.81},
      deltaAngleBodyFlu_rad=zeros(3), deltaVelocityBodyFlu_m_s={0.0,0.0,0.0981},
      deltaPositionBodyFlu_m={0.0,0.0,0.0004905}, deltaQuaternionBodyFlu={1.0,0.0,0.0,0.0},
      integrationTime_s=0.01, gyroscopeBiasLinearizationBodyFlu_rad_s=zeros(3),
      accelerometerBiasLinearizationBodyFlu_m_s2=zeros(3),
      deltaRotationGyroscopeBiasJacobian_s=-0.01*identity(3),
      deltaVelocityGyroscopeBiasJacobian_m=zeros(3,3),
      deltaVelocityAccelerometerBiasJacobian_s=-0.01*identity(3),
      deltaPositionGyroscopeBiasJacobian_m_s=zeros(3,3),
      deltaPositionAccelerometerBiasJacobian_s2=-0.00005*identity(3));
    if first then
      state := Estimation.StrapdownINS.UKF.initialize(zeros(3),zeros(3),{1.0,0.0,0.0,0.0},zeros(3),zeros(3),
        Estimation.StrapdownINS.InitialVariances(position_m2=fill(1.0,3),velocity_m2_s2=fill(1.0,3),
          attitude_rad2=fill(0.25,3),gyroscopeBias_rad2_s2=fill(1e-4,3),accelerometerBias_m2_s4=fill(0.01,3)));
      success := true;
    elseif preintegrated then
      (state,success) := Estimation.StrapdownINS.UKF.predictPreintegrated(
        Estimation.StrapdownINS.UKF.State(positionWorldEnu_m=previous.positionWorldEnu_m,velocityWorldEnu_m_s=previous.velocityWorldEnu_m_s,
          quaternionWorldBody=previous.quaternionWorldBody,gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
          accelerometerBiasBodyFlu_m_s2=previous.accelerometerBiasBodyFlu_m_s2,covariance=previous.covariance),
        imu,{0.0,0.0,-9.81},Estimation.StrapdownINS.ProcessNoise(gyroscope_rad2_s=identity(3)*1e-4,
          accelerometer_m2_s3=identity(3)*0.03,gyroscopeBias_rad2_s3=identity(3)*1e-10,
          accelerometerBias_m2_s5=identity(3)*1e-6));
    else
      (state,success) := Estimation.StrapdownINS.UKF.predict(
        previous, zeros(3), {0.0,0.0,9.81}, {0.0,0.0,-9.81}, 0.01,
        Estimation.StrapdownINS.ProcessNoise(gyroscope_rad2_s=identity(3)*1e-4,
          accelerometer_m2_s3=identity(3)*0.03,gyroscopeBias_rad2_s3=identity(3)*1e-10,
          accelerometerBias_m2_s5=identity(3)*1e-6));
    end if;
    position := state.positionWorldEnu_m;
    velocity := state.velocityWorldEnu_m_s;
    quaternion := state.quaternionWorldBody;
    gyroBias := state.gyroscopeBiasBodyFlu_rad_s;
    accelBias := state.accelerometerBiasBodyFlu_m_s2;
    covariance := state.covariance;
 end run;
 algorithm
 when sample(0,0.01) then
 (position,velocity,quaternion,gyroBias,accelBias,covariance,success) := run(first, preintegrated,
   Estimation.StrapdownINS.UKF.State(positionWorldEnu_m=pre(position),velocityWorldEnu_m_s=pre(velocity),
     quaternionWorldBody=pre(quaternion),gyroscopeBiasBodyFlu_rad_s=pre(gyroBias),
     accelerometerBiasBodyFlu_m_s2=pre(accelBias),covariance=pre(covariance)));
 end when;
end UKFHoverCodegen;
