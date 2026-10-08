within Estimation.StrapdownINS;

function imuSampleFinite
  "True when every numeric field of an inertial sample is finite and usable"
  input Avionics.ImuSample imu;
  output Boolean finite;
algorithm
  // AFFIRMATIVE ADMISSION. The prediction path has no innovation gate in front
  // of it, so a single non-finite inertial sample propagates straight into the
  // nominal state and stays there: nothing downstream can contradict it, and
  // the filter never recovers. An unusable sample carries the same information
  // as an absent one, so the caller must treat a false result exactly as a
  // dropout, holding the state and growing the covariance.
  //
  // Stated as a condition the sample must PROVE rather than a rejection test,
  // because every comparison against NaN is false: a rejection predicate built
  // from `>` would evaluate false for NaN and admit it.
  //
  // The integration time is required strictly positive, not merely finite,
  // because the preintegration divides by it.
  finite := abs(imu.timestamp_s) < ESKF.FiniteMagnitudeLimit
    and imu.integrationTime_s > 0.0
    and imu.integrationTime_s < ESKF.FiniteMagnitudeLimit;
  for axis in 1:3 loop
    finite := finite
      and abs(imu.angularVelocityBodyFlu_rad_s[axis]) < ESKF.FiniteMagnitudeLimit
      and abs(imu.specificForceBodyFlu_m_s2[axis]) < ESKF.FiniteMagnitudeLimit
      and abs(imu.deltaAngleBodyFlu_rad[axis]) < ESKF.FiniteMagnitudeLimit
      and abs(imu.deltaVelocityBodyFlu_m_s[axis]) < ESKF.FiniteMagnitudeLimit
      and abs(imu.deltaPositionBodyFlu_m[axis]) < ESKF.FiniteMagnitudeLimit
      and abs(imu.gyroscopeBiasLinearizationBodyFlu_rad_s[axis])
        < ESKF.FiniteMagnitudeLimit
      and abs(imu.accelerometerBiasLinearizationBodyFlu_m_s2[axis])
        < ESKF.FiniteMagnitudeLimit;
    for column in 1:3 loop
      finite := finite
        and abs(imu.deltaRotationGyroscopeBiasJacobian_s[axis, column])
          < ESKF.FiniteMagnitudeLimit
        and abs(imu.deltaVelocityGyroscopeBiasJacobian_m[axis, column])
          < ESKF.FiniteMagnitudeLimit
        and abs(imu.deltaVelocityAccelerometerBiasJacobian_s[axis, column])
          < ESKF.FiniteMagnitudeLimit
        and abs(imu.deltaPositionGyroscopeBiasJacobian_m_s[axis, column])
          < ESKF.FiniteMagnitudeLimit
        and abs(imu.deltaPositionAccelerometerBiasJacobian_s2[axis, column])
          < ESKF.FiniteMagnitudeLimit;
    end for;
  end for;
  for component in 1:4 loop
    finite := finite
      and abs(imu.deltaQuaternionBodyFlu[component]) < ESKF.FiniteMagnitudeLimit;
  end for;
end imuSampleFinite;
