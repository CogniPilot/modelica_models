within Estimation.MultiSensorInvariant;

function nominalStateFinite
  "True when every published nominal state component is finite"
  input Real position[3];
  input Real velocity[3];
  input Real quaternion[4];
  output Boolean finite;
algorithm
  // Publication guard on the estimate boundary. With the affirmative
  // acceptance predicates in correctLinear and solveSPD no non-finite
  // value should reach here at all; this exists so that if one ever does
  // -- through a path not yet audited, or through non-finite IMU data on
  // the prediction side, which no gate covers -- consumers see
  // `estimate.valid = false` and fall back, instead of a controller
  // computing a NaN actuator command from a NaN position and a valid
  // flag. A navigation estimate that cannot be represented as a number
  // is not a valid navigation estimate.
  //
  // Cost, since this is flight code on a 1 kHz tick: ten `abs` and ten
  // compares, straight-line, no branches, no calls. Against the
  // O(TangentLength^3) covariance propagation already on the same tick
  // (15^3 = 3375 multiply-adds in the transition alone) it is under a
  // tenth of a percent, and it is constant-time, so the WCET grows by a
  // fixed ten operations rather than by anything data dependent.
  //
  // Only the nominal state is checked, not the 225-entry covariance:
  // the covariance is not published across this boundary, and a
  // non-finite covariance can only reach a consumer by first producing
  // a non-finite nominal state through a correction -- which the
  // acceptance predicate now blocks -- so checking it here would cost
  // 22x more for no additional coverage.
  finite := true;
  for i in 1:3 loop
    finite := finite
      and abs(position[i]) < FiniteMagnitudeLimit
      and abs(velocity[i]) < FiniteMagnitudeLimit;
  end for;
  for i in 1:4 loop
    finite := finite and abs(quaternion[i]) < FiniteMagnitudeLimit;
  end for;
end nominalStateFinite;
