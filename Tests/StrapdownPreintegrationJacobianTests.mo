within Tests;

model StrapdownPreintegrationJacobianTests
  "Derivatives of composed held and FOH increments, including the position channel"
  function compose
    input Integer caseIndex;
    input Boolean useFirstOrderHold;
    input Real bias[6];
    output Real pose[10];
    output Real J[9, 6];
  protected
    constant Real dt = 0.04;
    constant Real anchor[6] = {0.02, -0.01, 0.03, 0.1, -0.2, 0.05};
    Real baseRate[3];
    Real rateStart[3];
    Real rateEnd[3];
    Real forceStart[3];
    Real forceEnd[3];
    Real position[3];
    Real velocity[3];
    Real quaternion[4];
    Real rotationGyro[3, 3];
    Real velocityGyro[3, 3];
    Real velocityAccel[3, 3];
    Real positionGyro[3, 3];
    Real positionAccel[3, 3];
  algorithm
    baseRate := if caseIndex == 1 then zeros(3)
      elseif caseIndex == 2 then {2.475, 0.0, 0.0}
      elseif caseIndex == 3 then {2.525, 0.0, 0.0}
      else {8.0, -4.0, 2.0};
    pose := cat(1, {0.4, -0.2, 0.3}, {-0.3, 0.7, 0.1},
      LieGroups.SO3.Quat.exp_map({0.3, -0.2, 0.4}));
    J := zeros(9, 6);
    for sampleIndex in 1:3 loop
      rateStart := anchor[1:3] + baseRate
        + (if caseIndex == 4 then (sampleIndex - 1) * {0.7, 0.2, -0.4}
          else zeros(3));
      rateEnd := anchor[1:3] + baseRate
        + (if caseIndex == 4 then sampleIndex * {0.7, 0.2, -0.4}
          else zeros(3));
      forceStart := anchor[4:6] + {1.0, -2.0, 9.0}
        + (sampleIndex - 1) * {0.8, -0.5, 0.3};
      forceEnd := anchor[4:6] + {1.0, -2.0, 9.0}
        + sampleIndex * {0.8, -0.5, 0.3};
      (position, velocity, quaternion, rotationGyro,
       velocityGyro, velocityAccel, positionGyro, positionAccel) :=
        Estimation.StrapdownINS.preintegrateImuStep(
          pose[1:3], pose[4:6], pose[7:10],
          J[7:9, 1:3], J[4:6, 1:3], J[4:6, 4:6],
          J[1:3, 1:3], J[1:3, 4:6], rateEnd, forceEnd,
          bias[1:3], bias[4:6], dt, useFirstOrderHold, rateStart, forceStart);
      pose := cat(1, position, velocity, quaternion);
      J := cat(1, cat(2, positionGyro, positionAccel),
        cat(2, velocityGyro, velocityAccel), cat(2, rotationGyro, zeros(3, 3)));
    end for;
  end compose;

  function run
    output Boolean passed;
  protected
    constant Real epsilon = 1.0e-5;
    constant Real anchor[6] = {0.02, -0.01, 0.03, 0.1, -0.2, 0.05};
    Real nominal[10];
    Real plus[10];
    Real minus[10];
    Real J[9, 6];
    Real scratch[9, 6];
    Real numerical[9, 6];
    Real perturbation[6];
    Real inverseQuaternion[4];
  algorithm
    // Both holds: zero rotation, either side of the coefficient-series branch,
    // and a large three-axis varying input. A nonidentity initial pose and
    // three intervals exercise transport of all five accumulated bias blocks.
    for holdIndex in 1:2 loop
      for caseIndex in 1:4 loop
        (nominal, J) := compose(caseIndex, holdIndex == 2, anchor);
        inverseQuaternion := LieGroups.SO3.Quat.inverse(nominal[7:10]);
        for column in 1:6 loop
          perturbation := {if row == column then epsilon else 0.0 for row in 1:6};
          (plus, scratch) := compose(caseIndex, holdIndex == 2,
            anchor + perturbation);
          (minus, scratch) := compose(caseIndex, holdIndex == 2,
            anchor - perturbation);
          numerical[1:6, column] := (plus[1:6] - minus[1:6]) / (2.0 * epsilon);
          numerical[7:9, column] := (
            LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
              inverseQuaternion, plus[7:10]))
            - LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
              inverseQuaternion, minus[7:10]))) / (2.0 * epsilon);
        end for;
        assert(Tests.Assertions.maxAbsMatrix(J - numerical) < 2.0e-8,
          "Composed IMU bias Jacobian disagrees with central differences");
      end for;
    end for;
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Composed preintegration Jacobian tests did not complete");
end StrapdownPreintegrationJacobianTests;
