within Tests.HorizonChecks;

function incrementalResidual
  "Worst |incremental predictor - re-base from the exact epoch pose|, through
   Estimation.FusionHorizon.step"
  input Integer count(min = 1);
  input Real dt(unit = "s");
  input Integer deltasPerFusion(min = 1);
  input Integer horizonWindows(min = 1);
  input Real gravityWorldEnu_m_s2[3];
  output Real worst[3] "position [m], velocity [m/s], attitude [rad]";
protected
  Real incrementalPose[count + 1, 10];
  Integer incrementalCount[count];
  Boolean incrementalReady[count];
  Real rebasedPose[count + 1, 10];
  Integer rebasedCount[count];
  Boolean rebasedReady[count];
  Real attitudeError_rad[3];
  Real attitudeMagnitude_rad;
algorithm
  // The predictor's cheap path composes only the newest delta onto its own
  // previous answer; its expensive path folds the whole ring, composes the
  // carried live window and this tick's delta onto it, and reapplies the
  // result to the horizon pose. Handed the EXACT pose the cheap path was
  // standing on at the fusion instant, the two must be the same element.
  //
  // This runs the real state machine. The previous version of this check
  // called rebaseResidual with a zero correction, which compared one fold
  // against a stepwise composition of the SAME buffer and never entered
  // step.mo at all: the ring indices, the live-window carry, the release
  // bookkeeping and the incremental branch were all outside it. A window
  // folded over the wrong contiguous run of slots passed it unchanged.
  (incrementalPose, incrementalCount, incrementalReady) :=
    Tests.HorizonChecks.runHorizonArm(
      count, dt, deltasPerFusion, horizonWindows, gravityWorldEnu_m_s2,
      false);
  (rebasedPose, rebasedCount, rebasedReady) :=
    Tests.HorizonChecks.runHorizonArm(
      count, dt, deltasPerFusion, horizonWindows, gravityWorldEnu_m_s2,
      true, incrementalPose, incrementalCount, incrementalReady);
  worst := zeros(3);
  for k in 1:count + 1 loop
    worst[1] := max(worst[1],
      max(abs(rebasedPose[k, 1:3] - incrementalPose[k, 1:3])));
    worst[2] := max(worst[2],
      max(abs(rebasedPose[k, 4:6] - incrementalPose[k, 4:6])));
    attitudeError_rad := LieGroups.SO3.Quat.log_map(
      LieGroups.SO3.Quat.product(
        LieGroups.SO3.Quat.inverse(incrementalPose[k, 7:10]),
        rebasedPose[k, 7:10]));
    attitudeMagnitude_rad := sqrt(attitudeError_rad * attitudeError_rad);
    worst[3] := max(worst[3], attitudeMagnitude_rad);
  end for;
end incrementalResidual;
