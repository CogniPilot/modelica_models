within Estimation.MultiSensorInvariant;

record Tuning
  "Fixed filter tuning held constant across every estimator tick"
  Estimation.MultiSensorInvariant.NominalState initialState
    "Nominal state used when no aiding source can seed initialization";
  Estimation.MultiSensorInvariant.InitialVariances initialVariances;
  Estimation.MultiSensorInvariant.ProcessNoise processNoise;
  Estimation.MultiSensorInvariant.VarianceLimits varianceLimits;
  Real innovationGate
    "Per-degree-of-freedom NIS gate; non-positive disables";
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
end Tuning;
