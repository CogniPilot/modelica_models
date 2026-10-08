within SLAM.Inertial;
// Modelica owns numerical subdivision of one unchanged held IMU measurement.
// Each substep uses the current midpoint rotation and propagates the full joint
// covariance. A failed substep rolls back the whole acquisition interval.
function ES15PredictHeldInterval
  import ES15Dynamics = SLAM.Inertial.ES15Dynamics;
  import ES15HeldIntervalValid = SLAM.Inertial.ES15HeldIntervalValid;
  import ES15NominalPrediction = SLAM.Inertial.ES15NominalPrediction;
  import ES15TransitionNoise = SLAM.Inertial.ES15TransitionNoise;
  import RGBDProperRotationValue = SLAM.Localization.RGBDProperRotationValue;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;
  import SchmidtPredictCovariance = SLAM.Inertial.SchmidtPredictCovariance;

  input Real position[3]; input Real velocity[3]; input Real rotation[3,3];
  input Real accelBias[3]; input Real gyroBias[3];
  input Real covariance[15,15]; input Real crossCovariance[15,6];
  input Real referenceCovariance[6,6]; input Real referencePosition[3];
  input Real referenceRotation[3,3]; input Real referenceAvailable;
  input Real accel[3]; input Real gyro[3]; input Real gravity[3];
  input Real h; input Real density[12];
  output Real accepted;
  output Real transition[15,15]; output Real processCovariance[15,15];
  output Real nextPosition[3]; output Real nextVelocity[3]; output Real nextRotation[3,3];
  output Real nextCovariance[15,15]; output Real nextCrossCovariance[15,6];
  output Integer substeps;
protected
  constant Integer maximumSubsteps = 64;
  Real rate; Real requestedSteps; Real dt;
  Real force[3]; Real omega[3]; Real middleRotation[3,3];
  Real proposedPosition[3]; Real proposedVelocity[3]; Real proposedRotation[3,3];
  Real F[15,15]; Real G[15,12]; Real Phi[15,15]; Real Q[15,15];
  Real proposedCovariance[15,15]; Real proposedCross[15,6]; Real retainedReference[6,6];
  Real rawNoise[15,15]; Real nominalValid; Real jointAccepted;
  Boolean geometryValid; Boolean densityValid; Boolean running;
algorithm
  accepted := 0.0; substeps := 0;
  transition := identity(15); processCovariance := zeros(15,15);
  nextPosition := position; nextVelocity := velocity; nextRotation := rotation;
  nextCovariance := covariance; nextCrossCovariance := crossCovariance;
  rate := sqrt(sum((gyro-gyroBias).^2));
  requestedSteps := max(h/0.02,rate*h/0.1);
  running := ES15HeldIntervalValid(h) and requestedSteps >= 0.0
    and requestedSteps <= maximumSubsteps;
  if running then
    substeps := max(1,integer(ceil(requestedSteps)));
    // Division rounding must not place a substep just beyond either limit.
    if h/substeps > 0.02 or rate*(h/substeps) > 0.1 then substeps := substeps+1; end if;
    running := substeps <= maximumSubsteps;
    if running then
      dt := h/substeps;
      densityValid := true;
      for channel in 1:12 loop
        densityValid := densityValid and density[channel] >= 0.0 and density[channel] <= 1e6;
      end for;
      for step in 1:substeps loop
        if running then
          (force,omega,middleRotation,proposedRotation,proposedPosition,proposedVelocity,nominalValid)
            := ES15NominalPrediction.Predict(nextRotation,nextPosition,nextVelocity,
              accel,gyro,accelBias,gyroBias,gravity,dt);
          (F,G) := ES15Dynamics.Matrices(middleRotation,force,omega);
          // The joint propagator below owns Phi*P*Phi'; compute it only once.
          (Phi,Q) := ES15TransitionNoise(F,G,
            if nominalValid > 0.5 then dt else 0.0,density);
          geometryValid := RGBDProperRotationValue(nextRotation) > 0.5
            and RGBDProperRotationValue(proposedRotation) > 0.5;
          for axis in 1:3 loop
            geometryValid := geometryValid and abs(proposedPosition[axis]) <= 1e6
              and abs(proposedVelocity[axis]) <= 1e6 and abs(accelBias[axis]) <= 2.0
              and abs(gyroBias[axis]) <= 0.3 and abs(gravity[axis]) <= 1e3
              and (SLAMExactRealEqual(referenceAvailable,0.0) or abs(referencePosition[axis]) <= 1e6);
          end for;
          geometryValid := geometryValid and (SLAMExactRealEqual(referenceAvailable,0.0)
            or RGBDProperRotationValue(referenceRotation) > 0.5);
          (jointAccepted,proposedCovariance,proposedCross,retainedReference)
            := SchmidtPredictCovariance(nextCovariance,nextCrossCovariance,referenceCovariance,
              Phi,Q,referenceAvailable,if nominalValid > 0.5 and geometryValid and densityValid then 1.0 else 0.0);
          // The first step preserves the original one-step diagnostics exactly.
          if step == 1 then
            transition := Phi; processCovariance := Q;
          else
            transition := Phi*transition;
            rawNoise := Phi*processCovariance*transpose(Phi)+Q;
            processCovariance := 0.5*(rawNoise+transpose(rawNoise));
          end if;
          running := jointAccepted > 0.5;
          if running then
            nextPosition := proposedPosition; nextVelocity := proposedVelocity; nextRotation := proposedRotation;
            nextCovariance := proposedCovariance; nextCrossCovariance := proposedCross;
          end if;
        end if;
      end for;
    end if;
    if running then
      accepted := 1.0;
    else
      nextPosition := position; nextVelocity := velocity; nextRotation := rotation;
      nextCovariance := covariance; nextCrossCovariance := crossCovariance;
    end if;
  end if;
end ES15PredictHeldInterval;
