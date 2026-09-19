within Estimation.StrapdownINS.ESKF;

record Tuning
  "Fixed filter tuning held constant across every estimator tick"
  Estimation.StrapdownINS.ESKF.NominalState initialState
    "Nominal state used when no aiding source can seed initialization";
  Estimation.StrapdownINS.InitialVariances initialVariances;
  Estimation.StrapdownINS.ProcessNoise processNoise;
  Estimation.StrapdownINS.ESKF.VarianceLimits varianceLimits;
  Real innovationGate
    "Per-degree-of-freedom NIS gate; non-positive disables";
  Real localMagneticFieldWorldEnu_T[3];
  Real barometerBias_m;
  Real barometerBiasVariance_m2;
  Real maximumAidingDelay_s;
  Real minimumOpticalFlowQuality;
  Real minimumOpticalFlowGroundDistance_m;
  Real covarianceInflateWindow_s
    "Wall-clock time the anchor source has been unable to move the state
     after which the position and velocity covariance begins ramping
     toward the mission envelope";
  Real covarianceInflateTimeConstant_s
    "e-folding time of that ramp; bounds how far one tick can widen the
     gate, and so how far one correction can move the state";
  Real aidingDivergentWindow_s
    "Wall-clock time after which the anchor is reported divergent; a
     status signal only, never a withdrawal of estimate.valid";
  Real aidingStaleTimeout_s
    "How long a source may go without a fresh sample before it stops
     counting as available for anchor selection. Bridges pulsed sensor
     wiring, where `valid` is asserted only on the ticks that carry a
     fix, and demotes a source that is permanently valid but never fresh";
  Real aidingReseedWindow_s
    "Wall-clock time the anchor source has been unable to move the state
     after which, on the next fresh finite anchor sample, the estimator
     re-seeds position (and velocity from a GPS sample that reports it)
     from that sample and restores the position and velocity covariance
     to the initial variances. Non-positive disables re-seeding; when
     enabled it must exceed aidingDivergentWindow_s, and any unusable
     value disables only the re-seed while leaving the rest of the ladder
     in force";
  Real initialAlignmentWindow_s
    "Unbroken quasi-static time required before the filter aligns its
     initial attitude from the IMU; non-positive aligns on the first usable
     sample";
  Real initialAlignmentTimeout_s
    "Time with a usable IMU after which a pending alignment is taken on the
     current sample even though the vehicle never held still for the
     window; non-positive waits indefinitely";
  Real quietSpecificForceTolerance_m_s2
    "How far the specific-force magnitude may differ from gravity while the
     vehicle still counts as quasi-static";
  Real quietAngularRateLimit_rad_s
    "Angular-rate magnitude below which the vehicle counts as quasi-static";
  Real quietFilterTimeConstant_s
    "Time constant of the first-order low-pass the quasi-static test and the
     accelerometer alignment read the IMU through, so a noisy sensor still
     shows a resting vehicle; non-positive reads the raw samples";
  Real pseudoPositionVariance_m2
    "Per-axis variance of the synthetic hold-position measurement fused while
     no anchor source is live; non-positive disables it";
  Real zeroVelocityVariance_m2_s2
    "Per-axis variance of the synthetic zero-velocity measurement fused while
     quasi-static and unaided; non-positive disables it";
end Tuning;
