within Tests.RuleChecks;
function retrodictChainResiduals
  "Chained rules against the retrodict difference and against the transition construction"
  input Real age_s(unit = "s") "Aiding delay the chain is evaluated at";
  input Real step "Central-difference step";
  input Integer trials "Number of randomized states";
  output Real worst[4]
    "chain vs difference, chain vs transition, H vs constructed H, largest chain entry";
protected
  Real draw[21];
  Real position[3];
  Real velocity[3];
  Real quaternion[4];
  Real gyroscopeBias[3];
  Real accelerometerBias[3];
  Real angularVelocity[3];
  Real specificForce[3];
  Real gravity[3];
  Real chain[15, 15];
  Real difference[15, 15];
  Real constructed[15, 15];
  Real selector[3, 15];
  // Scalar accumulators: see the note in so3RuleResiduals.
  Real worstDifference;
  Real worstTransition;
  Real worstMeasurement;
  Real largestEntry;
algorithm
  worstDifference := 0.0;
  worstTransition := 0.0;
  worstMeasurement := 0.0;
  largestEntry := 0.0;
  gravity := {0.0, 0.0, -9.81};
  // The same row selector correctGpsPosition applies to currentToDelayed.
  selector := cat(2, identity(3), zeros(3, 12));
  for trial in 1:trials loop
    draw := Tests.RuleChecks.pseudoRandom(31337 * trial + 5, 21);
    position := 50.0 * draw[1:3];
    velocity := 10.0 * draw[4:6];
    quaternion := LieGroups.SO3.Quat.exp_map(2.0 * draw[7:9]);
    gyroscopeBias := 0.02 * draw[10:12];
    accelerometerBias := 0.2 * draw[13:15];
    angularVelocity := 1.5 * draw[16:18];
    specificForce := 3.0 * draw[19:21] + {0.0, 0.0, 9.81};

    chain := Tests.RuleChecks.retrodictChain(
      Estimation.StrapdownINS.ESKF.State(
        positionWorldEnu_m = position,
        velocityWorldEnu_m_s = velocity,
        quaternionWorldBody = quaternion,
        gyroscopeBiasBodyFlu_rad_s = gyroscopeBias,
        accelerometerBiasBodyFlu_m_s2 = accelerometerBias,
        covariance = zeros(15, 15)),
      angularVelocity, specificForce, gravity, age_s);
    difference := Tests.RuleChecks.fdRetrodictJacobian(
      Estimation.StrapdownINS.ESKF.State(
        positionWorldEnu_m = position,
        velocityWorldEnu_m_s = velocity,
        quaternionWorldBody = quaternion,
        gyroscopeBiasBodyFlu_rad_s = gyroscopeBias,
        accelerometerBiasBodyFlu_m_s2 = accelerometerBias,
        covariance = zeros(15, 15)),
      angularVelocity, specificForce, gravity, age_s, step);
    constructed := Estimation.StrapdownINS.ESKF.discreteTransition(
      Estimation.StrapdownINS.ESKF.continuousTransition(
        angularVelocity - gyroscopeBias,
        specificForce - accelerometerBias), -age_s);

    worstDifference := max(worstDifference,
      Tests.Assertions.maxAbsMatrix(chain - difference));
    worstTransition := max(worstTransition,
      Tests.Assertions.maxAbsMatrix(chain - constructed));
    worstMeasurement := max(worstMeasurement, Tests.Assertions.maxAbsMatrix(
      selector * chain - selector * constructed));
    largestEntry := max(largestEntry,
      Tests.Assertions.maxAbsMatrix(chain));
  end for;
  worst := {worstDifference, worstTransition, worstMeasurement, largestEntry};
end retrodictChainResiduals;
