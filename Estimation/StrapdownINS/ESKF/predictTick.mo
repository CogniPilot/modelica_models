within Estimation.StrapdownINS.ESKF;

function predictTick
  "Predict or hold one IMU interval and bound its covariance"
  input State previous;
  input Avionics.ImuSample imu;
  input Tuning tuning;
  input Real gravityWorldEnu_m_s2[3];
  input Real dt;
  input Boolean inflateCovarianceNow;
  input Boolean imuUsable;
  input Boolean imuPayloadFinite;
  input Boolean vehicleAtRest;
  output State working;
  output Boolean predictionAccepted;
  output Boolean stationaryImuCorrectionAccepted;
protected
  State prior;
  Covariance heldDynamics;
  Integer aidingOutcome;
  Real aidingNis;
algorithm
  predictionAccepted := false;
  stationaryImuCorrectionAccepted := false;
  prior := if inflateCovarianceNow then
    inflateStateCovariance(previous, tuning.varianceLimits,
      dt, tuning.covarianceInflateTimeConstant_s) else copyState(previous);
  if imuUsable then
    if vehicleAtRest and tuning.useStationaryImu then
      working := predictStationary(
        prior, imu.integrationTime_s, tuning.processNoise);
      (working, stationaryImuCorrectionAccepted, aidingOutcome, aidingNis) :=
        correctStationaryImu(working, imu, gravityWorldEnu_m_s2,
          tuning.processNoise, tuning.innovationGate, tuning.useSemiDirectBias);
    else
      working := predictPreintegrated(
        prior, imu, gravityWorldEnu_m_s2, tuning.processNoise);
    end if;
    predictionAccepted := true;
  elseif not imuPayloadFinite then
    if prior.useSquareRootCovariance then
      heldDynamics := continuousTransition(zeros(3), zeros(3));
      working := predictCovariance(prior, discreteTransition(heldDynamics, dt),
        heldDynamics, tuning.processNoise, dt);
    else
      working := withDenseCovariance(prior,
        holdCovariance(prior.covariance, dt, tuning.processNoise),
        discreteTransition(continuousTransition(zeros(3), zeros(3)), dt)
          * prior.barometerBiasCrossCovariance);
    end if;
  else
    // A valid held packet means the high-rate preintegrator is still
    // accumulating the next delta-angle/delta-velocity observation. It is
    // neither a new prediction nor an IMU dropout, so do not propagate the
    // nominal state or add process noise a second time.
    working := copyState(prior);
  end if;
  working := limitStateCovariance(working, tuning.varianceLimits);
end predictTick;
