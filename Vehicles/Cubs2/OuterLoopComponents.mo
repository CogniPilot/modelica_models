within Vehicles.Cubs2;

// SPDX-License-Identifier: Apache-2.0

package OuterLoopComponents
  "Reusable records and sampled controller blocks for the Sport Cub S2 outer loop"


function horizontalPart
  input Real v[3];
  output Real result[2];
algorithm
  result := {v[1], v[2]};
annotation(
  Inline=true);
end horizontalPart;

function horizontalDisplacement
  input Real position[3];
  input Real origin[3];
  output Real result[2];
algorithm
  result := {position[1] - origin[1], position[2] - origin[2]};
annotation(
  Inline=true);
end horizontalDisplacement;

record VehicleParameters
  Real g(unit="m/s2", min=1.0e-9) = 9.81 "standard gravity";
  Real mass(unit="kg", min=1.0e-9) = 0.063 "flight-tuned 2026-07-10; SportCubPlant sims at 0.065";
  Real thrustMax(unit="N", min=1.0e-9) = 0.30 "FixedWingPlant.thr_max";
  Real trimThrust(unit="N") = 0.12 "40% throttle cruise trim (2026-07-10)";
  Real envelopeDrag(unit="N") = 0.07 "cruise drag";
  Real weight(unit="N") = mass * g "aircraft weight";
  Real drag(unit="N") = envelopeDrag "drag estimate";
end VehicleParameters;

record FlightState
  "Estimated aircraft state carried as one sampled value"
  Real position_m[3];
  Real euler_rad[3];
  Real velocity_m_s[3];
  Real speed(unit="m/s");
  Real flightPathAngle(unit="rad");
  Real acceleration_m_s2(unit="m/s2");
  Real eulerRate_rad_s[3];
end FlightState;

connector FlightStateInput = input FlightState
  "Typed consumer port for one sampled flight-state value";
connector FlightStateOutput = output FlightState
  "Single-producer port for one sampled flight-state value";

record GuidanceSetpoints
  "Cohesive route-guidance demand"
  Real speed(unit="m/s");
  Real flightPathAngle(unit="rad");
  Real heading(unit="rad");
  Real acceleration(unit="m/s2");
end GuidanceSetpoints;

connector GuidanceSetpointsInput = input GuidanceSetpoints
  "Typed consumer port for one sampled guidance demand";
connector GuidanceSetpointsOutput = output GuidanceSetpoints
  "Single-producer port for one sampled guidance demand";

record TecsCommands
  "TECS pitch and thrust command value"
  Real pitch(unit="rad");
  Real thrust(unit="N");
  Real pitchPreview(unit="rad");
  Real thrustPreview(unit="N");
end TecsCommands;

connector TecsCommandsInput = input TecsCommands
  "Typed consumer port for one sampled TECS command";
connector TecsCommandsOutput = output TecsCommands
  "Single-producer port for one sampled TECS command";

record StabilizerCommands
  "Normalized command sent to the proprietary onboard stabilizer"
  Real normalized[4]
    "{roll, pitch, yaw, throttle}";
end StabilizerCommands;

connector StabilizerCommandsOutput = output StabilizerCommands
  "Single-producer port for one sampled onboard-stabilizer command";
connector StabilizerCommandsInput = input StabilizerCommands
  "Typed consumer port for one sampled onboard-stabilizer command";

record RouteParameters
  Integer nSegments = 6 "flyable segments between route points";
  Real waypoints[7, 3] = [
    0.0, 0.0, 0.0;
    -8.0, -8.0, 3.0;
    -8.0, 2.0, 3.0;
    18.0, 2.0, 3.0;
    18.0, -8.0, 3.0;
    5.0, -8.0, 3.0;
    -8.0, -8.0, 3.0] "route point rows are [x, y, z] [m]";
  Real cruiseSpeed(unit="m/s") = 4.5;
  Real altitudeToFlightPathGain = 2.0;
  Real altitudeLookaheadDistance(unit="m") = 8.0;
  Real flightPathAngleLimit(unit="rad") = 0.12;
  Real speedToAccelerationGain = 1.0;
  Real crossTrackSteeringDistance(unit="m") = 4.25
    "atan steering distance; 45 deg correction occurs when |cross-track| = d";
  Real waypointSwitchingDistance(unit="m") = 4.0
    "advance when remaining along-track distance enters the endpoint guard";
end RouteParameters;

record TecsParameters
  Real thrustKp = 0.05 "P on energy-rate error";
  Real thrustKi = 0.004854133294500267
    "10 s unit-error ramp to 10% throttle integral authority";
  Real thrustIntegralLimit(unit="N") = 0.03
    "maximum thrust contribution from integral action";
  Real pitchKp = 0.075;
  Real pitchKi = 0.0;
  Real pitchIntegralLimit(unit="rad") = 0.0;
  Real trackingTime(unit="s", min=1.0e-9) = 0.2
    "anti-windup tracking time constant";
  Real pitchCommandLimit(unit="rad") = 0.20943951023931953;
  Real turnThrustGain = 0.5 "thrust FF per unit load-factor excess";
  Real turnPitchGain = 0.0 "pitch FF [rad] per unit load-factor excess";
end TecsParameters;

record AttitudeParameters
  Real trimElevator = 0.0
    "manual cruise trim from 2026-07-09/10 level-ish flight segments";
  Real stabilizerCommand(unit="us") = 2000.0;
  Real pitchCommandToElevatorGain = 1.0 / 0.5235987755982988
    "S2 pitch stick gain: full elevator stick maps to 30 deg (bench-measured 2026-07-10)";
  // Full aileron stick is 45 deg of bank on the real S2.
  Real rollCommandToAileronGain = 1.0 / 0.7853981633974483
    "S2 bank stick gain: full aileron stick maps to 45 deg";
  Real rollLimit(unit="rad") = 0.5235987755982988
    "30 deg bank command saturation";
  Real rollRateLimit(unit="rad/s") = 2.0943951023931953;
end AttitudeParameters;

block StateEstimator
  parameter Real dt(unit="s", min=1.0e-9) = 0.02;
  parameter Real filterCutoffHz(unit="Hz") = 10.0;
  parameter Real velocityFilterCutoffHz(unit="Hz") = 1.5;
  parameter Real accelFilterCutoffHz(unit="Hz") = 0.4;
  parameter Real freshnessDistance(unit="m", min=1.0e-12) = 1.0e-5
    "Pose-equivalent motion that gives a packet full correction weight";
  parameter Real attitudeFreshnessLength(unit="m") = 0.1
    "Length scale converting attitude motion into pose-equivalent motion";
  parameter Real positionCorrectionSpeedLimit(unit="m/s") = 10.0;
  parameter Real attitudeCorrectionRateLimit(unit="rad/s") = 4.0;
  constant Real pi = 3.141592653589793;

  input Real position_m[3];
  input Real euler_rad[3];
  input Real velocity_m_s[3];
  input Real eulerRate_rad_s[3];
  FlightStateOutput estimate(
    position_m(each start=0.0),
    euler_rad(each start=0.0),
    velocity_m_s(each start=0.0),
    speed(start=0.0),
    flightPathAngle(start=0.0),
    acceleration_m_s2(start=0.0),
    eulerRate_rad_s(each start=0.0));
  discrete output Real measurementWeight(start=0.0)
    "Numeric correction weight; zero for a repeated mocap packet";

protected
  discrete Real previousMeasurementPosition_m[3](each start=0.0);
  discrete Real previousMeasurementEuler_rad[3](each start=0.0);
  discrete Real previousMeasurementVelocity_m_s[3](each start=0.0);
  discrete Real previousMeasurementEulerRate_rad_s[3](each start=0.0);
  discrete Real measurementPositionDelta_m[3];
  discrete Real measurementEulerDelta_rad[3];
  discrete Real measurementVelocityDelta_m_s[3];
  discrete Real measurementEulerRateDelta_rad_s[3];
  discrete Real predictedPosition_m[3];
  discrete Real predictedEuler_rad[3];
  discrete Real positionInnovation_m[3];
  discrete Real attitudeInnovation_rad[3];
  discrete Real sampleMotion_m;
  discrete Real positionInnovationWeight;
  discrete Real attitudeInnovationWeight;
  discrete Real measuredSpeed;
  discrete Real measuredFlightPathAngle;
  discrete Real filterSampleWeight;
  discrete Real velocitySampleWeight;
  discrete Real accelSampleWeight;
  discrete Real rawAcceleration_m_s2;

algorithm
  when sample(0.0, dt) then
    filterSampleWeight := 1.0 - exp(-2.0 * pi * filterCutoffHz * dt);
    velocitySampleWeight := 1.0 - exp(-2.0 * pi * velocityFilterCutoffHz * dt);
    accelSampleWeight := 1.0 - exp(-2.0 * pi * accelFilterCutoffHz * dt);

    measurementPositionDelta_m :=
      position_m - pre(previousMeasurementPosition_m);
    measurementVelocityDelta_m_s :=
      velocity_m_s - pre(previousMeasurementVelocity_m_s);
    for axis in 1:3 loop
      measurementEulerDelta_rad[axis] := MathUtilities.wrapAngle(
        euler_rad[axis] - pre(previousMeasurementEuler_rad[axis]));
      measurementEulerRateDelta_rad_s[axis] :=
        eulerRate_rad_s[axis] - pre(previousMeasurementEulerRate_rad_s[axis]);
    end for;
    sampleMotion_m :=
      MathUtilities.norm3(measurementPositionDelta_m)
      + attitudeFreshnessLength * MathUtilities.norm3(measurementEulerDelta_rad)
      + dt * MathUtilities.norm3(measurementVelocityDelta_m_s)
      + dt * attitudeFreshnessLength
        * MathUtilities.norm3(measurementEulerRateDelta_rad_s);
    measurementWeight :=
      MathUtilities.clip(sampleMotion_m / freshnessDistance, 0.0, 1.0);

    predictedPosition_m :=
      pre(estimate.position_m) + dt * pre(estimate.velocity_m_s);
    for axis in 1:3 loop
      predictedEuler_rad[axis] := MathUtilities.wrapAngle(
        pre(estimate.euler_rad[axis])
          + dt * pre(estimate.eulerRate_rad_s[axis]));
      attitudeInnovation_rad[axis] := MathUtilities.wrapAngle(
        euler_rad[axis] - predictedEuler_rad[axis]);
    end for;
    positionInnovation_m := position_m - predictedPosition_m;
    positionInnovationWeight := MathUtilities.clip(
      positionCorrectionSpeedLimit * dt
        / max(MathUtilities.norm3(positionInnovation_m), 1.0e-12),
      0.0,
      1.0);
    attitudeInnovationWeight := MathUtilities.clip(
      attitudeCorrectionRateLimit * dt
        / max(MathUtilities.norm3(attitudeInnovation_rad), 1.0e-12),
      0.0,
      1.0);

    estimate.position_m := predictedPosition_m
      + filterSampleWeight * measurementWeight
        * positionInnovationWeight * positionInnovation_m;
    for axis in 1:3 loop
      estimate.euler_rad[axis] := MathUtilities.wrapAngle(
        predictedEuler_rad[axis]
          + filterSampleWeight * measurementWeight
            * attitudeInnovationWeight * attitudeInnovation_rad[axis]);
    end for;
    estimate.velocity_m_s := MathUtilities.lowPass3(
      velocity_m_s,
      pre(estimate.velocity_m_s),
      velocitySampleWeight * measurementWeight);
    estimate.eulerRate_rad_s := MathUtilities.lowPass3(
      eulerRate_rad_s,
      pre(estimate.eulerRate_rad_s),
      filterSampleWeight * measurementWeight);

    measuredSpeed := MathUtilities.norm3(estimate.velocity_m_s);
    measuredFlightPathAngle := asin(MathUtilities.clip(
      estimate.velocity_m_s[3] / max(measuredSpeed, 1.0e-5), -1.0, 1.0));
    estimate.speed := measuredSpeed;
    estimate.flightPathAngle := measuredFlightPathAngle;
    rawAcceleration_m_s2 := (estimate.speed - pre(estimate.speed)) / dt;
    estimate.acceleration_m_s2 := MathUtilities.lowPass(
      rawAcceleration_m_s2,
      pre(estimate.acceleration_m_s2),
      accelSampleWeight);

    previousMeasurementPosition_m := position_m;
    previousMeasurementEuler_rad := euler_rad;
    previousMeasurementVelocity_m_s := velocity_m_s;
    previousMeasurementEulerRate_rad_s := eulerRate_rad_s;
  end when;
end StateEstimator;

partial block RouteGuidanceInterface
  parameter Real dt(unit="s", min=1.0e-9) = 0.02;
  parameter RouteParameters route = RouteParameters();

  FlightStateInput estimate;

  discrete output Integer currentWaypoint(min=1, max=6, start=1);
  GuidanceSetpointsOutput setpoints;
end RouteGuidanceInterface;

block CrossTrackGuidance
  "Straight-segment guidance with nonlinear cross-track correction"
  extends RouteGuidanceInterface;

protected
  discrete Integer activeWaypoint(min=1, max=6, start=1);
  discrete Integer segmentEndIndex(min=2, max=7, start=2);
  discrete Real segmentStart[3];
  discrete Real segmentEnd[3];
  discrete Real segmentVector[3];
  discrete Real horizontalSegmentVector[2];
  discrete Real horizontalSegmentLength;
  discrete Real segmentHeading;
  discrete Real segmentUnit[2];
  discrete Real crossTrackUnit[2];
  discrete Real positionFromSegmentStart[2];
  discrete Real alongTrackDistance;
  discrete Real remainingAlongTrackDistance;
  discrete Real pathProgress;
  discrete Real pathAltitude;
  discrete Real altitudeError;
  discrete Real crossTrackError;
  discrete Real steeringCorrection;

algorithm
  when sample(0.0, dt) then
    activeWaypoint := pre(currentWaypoint);
    segmentEndIndex := pre(currentWaypoint) + 1;
    if pre(currentWaypoint) == 1 then
      segmentStart := route.waypoints[1, :];
      segmentEnd := route.waypoints[2, :];
    elseif pre(currentWaypoint) == 2 then
      segmentStart := route.waypoints[2, :];
      segmentEnd := route.waypoints[3, :];
    elseif pre(currentWaypoint) == 3 then
      segmentStart := route.waypoints[3, :];
      segmentEnd := route.waypoints[4, :];
    elseif pre(currentWaypoint) == 4 then
      segmentStart := route.waypoints[4, :];
      segmentEnd := route.waypoints[5, :];
    elseif pre(currentWaypoint) == 5 then
      segmentStart := route.waypoints[5, :];
      segmentEnd := route.waypoints[6, :];
    else
      segmentStart := route.waypoints[6, :];
      segmentEnd := route.waypoints[7, :];
    end if;

    segmentVector := segmentEnd - segmentStart;
    horizontalSegmentVector := horizontalPart(segmentVector);
    horizontalSegmentLength := max(MathUtilities.norm2(horizontalSegmentVector), 1e-6);
    segmentHeading := atan2(horizontalSegmentVector[2], horizontalSegmentVector[1]);
    segmentUnit := horizontalSegmentVector / horizontalSegmentLength;
    crossTrackUnit := {-segmentUnit[2], segmentUnit[1]};
    positionFromSegmentStart :=
      horizontalDisplacement(estimate.position_m, segmentStart);
    alongTrackDistance := positionFromSegmentStart * segmentUnit;
    remainingAlongTrackDistance :=
      max(0.0,
          horizontalSegmentLength
          - MathUtilities.clip(alongTrackDistance, 0.0, horizontalSegmentLength));
    pathProgress :=
      MathUtilities.clip(alongTrackDistance / horizontalSegmentLength, 0.0, 1.0);
    pathAltitude :=
      segmentStart[3] + pathProgress * (segmentEnd[3] - segmentStart[3]);
    altitudeError := pathAltitude - estimate.position_m[3];
    crossTrackError := positionFromSegmentStart * crossTrackUnit;
    steeringCorrection :=
      atan2(-crossTrackError, max(route.crossTrackSteeringDistance, 1e-6));

    setpoints.speed := route.cruiseSpeed;
    setpoints.flightPathAngle :=
      MathUtilities.clip(atan2(route.altitudeToFlightPathGain * altitudeError,
                 max(route.altitudeLookaheadDistance, 1e-6)),
           -route.flightPathAngleLimit,
           route.flightPathAngleLimit);
    setpoints.heading := MathUtilities.wrapAngle(segmentHeading + steeringCorrection);
    setpoints.acceleration :=
      route.speedToAccelerationGain * (setpoints.speed - estimate.speed);

    if remainingAlongTrackDistance < route.waypointSwitchingDistance then
      if pre(currentWaypoint) >= route.nSegments then
        currentWaypoint := 1;
      else
        currentWaypoint := pre(currentWaypoint) + 1;
      end if;
    else
      currentWaypoint := pre(currentWaypoint);
    end if;
  end when;
end CrossTrackGuidance;

record TecsState "Numeric state of the sampled TECS transition"
  Real thrustIntegralContribution(unit="N");
  Real thrustError;
  Real thrustDerivative;
  Real pitchIntegralContribution(unit="rad");
  Real pitchError;
  Real pitchDerivative;
end TecsState;

record TecsDiagnostics "Proof and tuning outputs from one TECS transition"
  Real boundedAcceleration(unit="m/s2");
  Real energyRateError;
  Real energyDistributionError;
  Real unconstrainedThrust(unit="N");
  Real unconstrainedPitch(unit="rad");
end TecsDiagnostics;

function tecsTransition
  "Pure fixed-dimensional total-energy controller transition"
  input Real dt(unit="s");
  input VehicleParameters vehicle;
  input TecsParameters tecs;
  input Control.PidParameters thrustPid;
  input Control.PidParameters pitchPid;
  input Control.PidCoefficients pidCoefficients;
  input GuidanceSetpoints setpoints;
  input FlightState estimate;
  input TecsState previous;
  input Real previousAppliedThrust(unit="N");
  input Real previousAppliedPitch(unit="rad");
  output TecsState next;
  output TecsCommands commands;
  output TecsDiagnostics diagnostics;

protected
  Real accelerationMin(unit="m/s2");
  Real accelerationMax(unit="m/s2");
  Real accelerationCommandOverG;
  Real accelerationEstimateOverG;
  Real totalEnergyRateCommand;
  Real totalEnergyRateEstimate;
  Real energyDistributionCommand;
  Real energyDistributionEstimate;
  Real loadFactorExcess;
  Real feedforwardThrust(unit="N");
  Real feedforwardPitch(unit="rad");
  Control.PidState nextThrustState;
  Control.PidState nextPitchState;
  Control.PidResult thrustFeedback;
  Control.PidResult pitchFeedback;

algorithm
  accelerationMin := -vehicle.drag / vehicle.mass;
  accelerationMax := (vehicle.thrustMax - vehicle.drag) / vehicle.mass;
  diagnostics.boundedAcceleration := MathUtilities.clip(
    setpoints.acceleration, accelerationMin, accelerationMax);
  accelerationCommandOverG := diagnostics.boundedAcceleration / vehicle.g;
  accelerationEstimateOverG := estimate.acceleration_m_s2 / vehicle.g;
  // NASA CR-178285, Sec. 2.1, Eq. (3): E_dot_s / V = gamma + V_dot / g.
  totalEnergyRateCommand := setpoints.flightPathAngle + accelerationCommandOverG;
  totalEnergyRateEstimate := estimate.flightPathAngle + accelerationEstimateOverG;
  // The distribution channel subtracts the normalized acceleration term so
  // pitch transfers energy between flight path and speed without changing
  // the total-energy channel.
  energyDistributionCommand := setpoints.flightPathAngle - accelerationCommandOverG;
  energyDistributionEstimate := estimate.flightPathAngle - accelerationEstimateOverG;

  loadFactorExcess := 1.0 / max(cos(estimate.euler_rad[1]), 0.5) - 1.0;
  feedforwardThrust := vehicle.trimThrust
    + vehicle.weight
      * (totalEnergyRateCommand + tecs.turnThrustGain * loadFactorExcess);
  feedforwardPitch := tecs.turnPitchGain * loadFactorExcess;
  diagnostics.energyRateError :=
    totalEnergyRateCommand - totalEnergyRateEstimate;
  diagnostics.energyDistributionError :=
    energyDistributionCommand - energyDistributionEstimate;

  (nextThrustState, thrustFeedback) := Control.pidTransition(
    thrustPid,
    pidCoefficients,
    Control.PidState(
      previous.thrustIntegralContribution,
      previous.thrustError,
      previous.thrustDerivative),
    totalEnergyRateCommand,
    totalEnergyRateEstimate,
    previousAppliedThrust - feedforwardThrust);
  (nextPitchState, pitchFeedback) := Control.pidTransition(
    pitchPid,
    pidCoefficients,
    Control.PidState(
      previous.pitchIntegralContribution,
      previous.pitchError,
      previous.pitchDerivative),
    energyDistributionCommand,
    energyDistributionEstimate,
    previousAppliedPitch - feedforwardPitch);

  next.thrustIntegralContribution := nextThrustState.integralContribution;
  next.thrustError := nextThrustState.error;
  next.thrustDerivative := nextThrustState.derivative;
  next.pitchIntegralContribution := nextPitchState.integralContribution;
  next.pitchError := nextPitchState.error;
  next.pitchDerivative := nextPitchState.derivative;

  commands.thrustPreview := MathUtilities.clip(
    feedforwardThrust + thrustFeedback.preview,
    0.0,
    vehicle.thrustMax);
  commands.pitchPreview := MathUtilities.clip(
    feedforwardPitch + pitchFeedback.preview,
    -tecs.pitchCommandLimit,
    tecs.pitchCommandLimit);

  diagnostics.unconstrainedThrust :=
    feedforwardThrust + thrustFeedback.unconstrainedCommand;
  commands.thrust := MathUtilities.clip(
    diagnostics.unconstrainedThrust, 0.0, vehicle.thrustMax);
  diagnostics.unconstrainedPitch :=
    feedforwardPitch + pitchFeedback.unconstrainedCommand;
  commands.pitch := MathUtilities.clip(
    diagnostics.unconstrainedPitch,
    -tecs.pitchCommandLimit,
    tecs.pitchCommandLimit);
  annotation(Documentation(info="<html>
    <p>The energy coordinates follow K. R. Bruce,
    <a href=\"https://ntrs.nasa.gov/api/citations/19870017485/downloads/19870017485.pdf\">
    NASA B737 Flight Test Results of the Total Energy Control System,
    NASA CR-178285</a>, Section 2.1, report page 6. Equation (3) gives
    <code>E_dot_s/V = gamma + V_dot/g</code>; Equation (4) gives
    <code>T_req = W (gamma + V_dot/g) + D</code>. The total-energy-rate
    and energy-distribution PI laws immediately following Equation (4) are
    represented by the two calls to <code>Control.pidTransition</code>.</p>
    <p>The trim/turn terms are explicit feedforward outside those PID maps.
    Command limits and applied-command tracking are CUBS2 adaptations and are
    not presented as equations from the NASA report.</p>
  </html>"));
end tecsTransition;

// TECS follows NASA CR-178285: thrust controls total energy rate while pitch
// redistributes energy between flight-path and speed.
block TECSController
  parameter Real dt(unit="s", min=1.0e-9) = 0.02;
  parameter VehicleParameters vehicle = VehicleParameters();
  parameter TecsParameters tecs = TecsParameters();
  final parameter Control.PidParameters thrustPid = Control.PidParameters(
    samplePeriod=dt,
    kp=vehicle.weight * tecs.thrustKp,
    ki=vehicle.weight * tecs.thrustKi,
    kd=0.0,
    integralLimit=tecs.thrustIntegralLimit,
    commandMin=-vehicle.thrustMax,
    commandMax=vehicle.thrustMax,
    trackingTime=tecs.trackingTime);
  final parameter Control.PidParameters pitchPid = Control.PidParameters(
    samplePeriod=dt,
    kp=tecs.pitchKp,
    ki=tecs.pitchKi,
    kd=0.0,
    integralLimit=tecs.pitchIntegralLimit,
    commandMin=-tecs.pitchCommandLimit,
    commandMax=tecs.pitchCommandLimit,
    trackingTime=tecs.trackingTime);
  final parameter Control.PidCoefficients pidCoefficients =
    Control.PidCoefficients(
      derivativeWeight=1.0,
      trackingCoefficient=dt / max(tecs.trackingTime, 1.0e-9));

  GuidanceSetpointsInput setpoints;
  FlightStateInput estimate;

  discrete output Real boundedAcceleration(unit="m/s2", start=0.0);
  discrete output Real energyRateError(start=0.0);
  discrete output Real thrustIntegralContribution(unit="N", start=0.0);
  discrete output Real unconstrainedThrustCommand(unit="N", start=0.0);
  TecsCommandsOutput commands;
  discrete output Real energyDistributionError(start=0.0);
  discrete output Real pitchIntegralContribution(unit="rad", start=0.0);
  discrete output Real unconstrainedPitchCommand(unit="rad", start=0.0);

protected
  TecsState nextState;
  TecsCommands nextCommands;
  TecsDiagnostics diagnostics;
  discrete Real thrustErrorDerivative(start=0.0);
  discrete Real pitchErrorDerivative(start=0.0);

algorithm
  when sample(0.0, dt) then
    (nextState, nextCommands, diagnostics) := tecsTransition(
      dt,
      vehicle,
      tecs,
      thrustPid,
      pitchPid,
      pidCoefficients,
      setpoints,
      estimate,
      TecsState(
        pre(thrustIntegralContribution),
        pre(energyRateError),
        pre(thrustErrorDerivative),
        pre(pitchIntegralContribution),
        pre(energyDistributionError),
        pre(pitchErrorDerivative)),
      pre(commands.thrust),
      pre(commands.pitch));
    thrustIntegralContribution := nextState.thrustIntegralContribution;
    pitchIntegralContribution := nextState.pitchIntegralContribution;
    thrustErrorDerivative := nextState.thrustDerivative;
    pitchErrorDerivative := nextState.pitchDerivative;
    commands.thrust := nextCommands.thrust;
    commands.pitch := nextCommands.pitch;
    commands.thrustPreview := nextCommands.thrustPreview;
    commands.pitchPreview := nextCommands.pitchPreview;
    boundedAcceleration := diagnostics.boundedAcceleration;
    energyRateError := diagnostics.energyRateError;
    energyDistributionError := diagnostics.energyDistributionError;
    unconstrainedThrustCommand := diagnostics.unconstrainedThrust;
    unconstrainedPitchCommand := diagnostics.unconstrainedPitch;
  end when;
end TECSController;

block AttitudeController
  parameter Real dt(unit="s", min=1.0e-9) = 0.02;
  parameter VehicleParameters vehicle = VehicleParameters();
  parameter AttitudeParameters params = AttitudeParameters();
  parameter Control.PidParameters headingPid =
    Control.PidParameters(samplePeriod=dt, kp=0.5, ki=0.0,
                          kd=0.5, integralLimit=0.0,
                          commandMin=-params.rollLimit,
                          commandMax=params.rollLimit);
  // Closed pitch loop: the plant's stick-to-pitch response is not trusted,
  // so the open-loop gain rides along as feedforward and the PID trims it.
  parameter Control.PidParameters pitchPid =
    Control.PidParameters(samplePeriod=dt, kp=2.3, ki=0.0, kd=0.0,
                          integralLimit=0.0,
                          commandMin=-1.0, commandMax=1.0);

  final parameter Control.PidCoefficients headingCoefficients =
    Control.PidCoefficients(
      derivativeWeight=if headingPid.derivativeCutoffHz > 0.0 then
        1.0 - exp(-2.0 * 3.141592653589793
          * headingPid.derivativeCutoffHz * dt) else 1.0,
      trackingCoefficient=dt / max(headingPid.trackingTime, 1.0e-9));
  final parameter Control.PidCoefficients pitchCoefficients =
    Control.PidCoefficients(
      derivativeWeight=if pitchPid.derivativeCutoffHz > 0.0 then
        1.0 - exp(-2.0 * 3.141592653589793
          * pitchPid.derivativeCutoffHz * dt) else 1.0,
      trackingCoefficient=dt / max(pitchPid.trackingTime, 1.0e-9));

  GuidanceSetpointsInput setpoints;
  FlightStateInput estimate;
  TecsCommandsInput tecsCommands;

  StabilizerCommandsOutput commands;
  discrete output Real rollCommand(start=0.0);
  discrete output Real rollCommandPreview(start=0.0)
    "integral-free bank; what guidance would fly while disengaged";
  output Real courseError;

protected
  discrete Real rollCommandState(start=0.0);
  discrete Real unlimitedRollCommand;
  discrete Real headingIntegral(start=0.0);
  discrete Real headingError(start=0.0);
  discrete Real headingDerivative(start=0.0);
  discrete Real headingUnconstrained(start=0.0);
  discrete Real headingCommand(start=0.0);
  discrete Real headingPreview(start=0.0);
  discrete Real pitchIntegral(start=0.0);
  discrete Real pitchError(start=0.0);
  discrete Real pitchDerivative(start=0.0);
  discrete Real pitchUnconstrained(start=0.0);
  discrete Real pitchCommand(start=0.0);

equation
  courseError = -MathUtilities.wrapAngle(
    setpoints.heading
    - atan2(estimate.velocity_m_s[2], estimate.velocity_m_s[1]));

algorithm
  when sample(0.0, dt) then
    // Keep both PID transitions inside this single guarded producer. Scalar
    // updates avoid a nested multi-output function transaction, which is not
    // representable by the checked eFMI/Solve-IR event contract.
    headingError := courseError;
    headingDerivative := pre(headingDerivative)
      + headingCoefficients.derivativeWeight
        * ((headingError - pre(headingError)) / dt
          - pre(headingDerivative));
    headingUnconstrained := headingPid.kp * headingError
      + pre(headingIntegral) + headingPid.kd * headingDerivative;
    headingCommand := MathUtilities.clip(
      headingUnconstrained, headingPid.commandMin, headingPid.commandMax);
    headingPreview := MathUtilities.clip(
      headingPid.kp * headingError + headingPid.kd * headingDerivative,
      headingPid.commandMin, headingPid.commandMax);
    headingIntegral := MathUtilities.clip(
      pre(headingIntegral) + headingPid.ki * headingError * dt
        + headingCoefficients.trackingCoefficient
          * (pre(headingCommand) - headingUnconstrained),
      -headingPid.integralLimit, headingPid.integralLimit);

    pitchError := tecsCommands.pitch - estimate.euler_rad[2];
    pitchDerivative := pre(pitchDerivative)
      + pitchCoefficients.derivativeWeight
        * ((pitchError - pre(pitchError)) / dt - pre(pitchDerivative));
    pitchUnconstrained := pitchPid.kp * pitchError
      + pre(pitchIntegral) + pitchPid.kd * pitchDerivative;
    pitchCommand := MathUtilities.clip(
      pitchUnconstrained, pitchPid.commandMin, pitchPid.commandMax);
    pitchIntegral := MathUtilities.clip(
      pre(pitchIntegral) + pitchPid.ki * pitchError * dt
        + pitchCoefficients.trackingCoefficient
          * (pre(pitchCommand) - pitchUnconstrained),
      -pitchPid.integralLimit, pitchPid.integralLimit);

    unlimitedRollCommand := headingCommand;
    rollCommandState :=
      MathUtilities.clip(
        MathUtilities.rateLimit(unlimitedRollCommand,
          pre(rollCommandState), params.rollRateLimit * dt),
        -params.rollLimit, params.rollLimit);
    rollCommand := rollCommandState;
    rollCommandPreview :=
      MathUtilities.clip(headingPreview, -params.rollLimit, params.rollLimit);
    // Assign the record array as one guarded producer. Besides expressing the
    // command atomically, this gives event schedulers a complete transaction
    // instead of four element writes to one array-valued variable.
    commands.normalized := {
      MathUtilities.clip(
        params.rollCommandToAileronGain * rollCommand, -1.0, 1.0),
      MathUtilities.clip(
        params.trimElevator
          + params.pitchCommandToElevatorGain * tecsCommands.pitch
          + pitchCommand,
        -1.0, 1.0),
      0.0,
      MathUtilities.clip(
        tecsCommands.thrust / vehicle.thrustMax, 0.0, 1.0)};
  end when;
end AttitudeController;

end OuterLoopComponents;
