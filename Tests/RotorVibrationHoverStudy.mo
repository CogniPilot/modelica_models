within Tests;

model RotorVibrationHoverStudy
  "Inertial mechanization error of the RDD2 plant hovering under rotor vibration"

  model HoverCase
    "One vibration point flown in hover with its IMU stream dead reckoned"
    constant Real pi = 2.0 * asin(1.0);
    parameter Boolean vibrationEnabled = true
      "False flies the same hover with no rotor content, which is the floor
       that every vibration case has to be read against";
    parameter Real vibrationAngularAmplitude_rad_s(unit = "rad/s") = 0.05
      "Per-rotor angular-rate vibration amplitude at hover trim";
    parameter Real vibrationSpecificForceAmplitude_m_s2(unit = "m/s2") = 1.0
      "Per-rotor lateral specific-force vibration amplitude at hover trim";
    parameter Real vehicleMass_kg(unit = "kg") = 2.0;
    parameter Real gravity_m_s2(unit = "m/s2") = 9.80665;
    parameter Real propellerThrustCoefficient_N_s2(unit = "N.s2") = 8.54858e-6
      "RDD2 propeller thrust per squared rotor speed";
    parameter Real hoverFundamentalTarget_Hz(unit = "Hz") = 0.0
      "Zero flies the RDD2 propeller. A positive value sizes a propeller that
       trims at that tone instead, which moves the vibration frequency
       without moving its amplitude, because the amplitude law is referenced
       to each case own hover speed";
    parameter Real thrustToWeightRatio(min = 1.2) = 2.0
      "Full-command thrust as a multiple of weight, held equal across the
       propeller cases so they share one control authority";
    parameter Boolean antiAliasFilter = true
      "Run the sensor low-pass that precedes sampling in the real part";
    parameter Real imuSamplePeriod_s(unit = "s") = 0.001
      "Raw IMU sampling interval, matching the vehicle composition rate";
    parameter Real initialGroundClearance_m(unit = "m") = 1.0
      "Start the case in the air so the study measures hover and not a
       landing-gear contact transient";
    parameter Real hoverAltitude_m(unit = "m") = 1.1
      "Hover hold altitude, set to the initial trim height";
    parameter Real altitudeGain_1_s2(unit = "1/s2") = 2.0;
    parameter Real climbRateGain_1_s(unit = "1/s") = 3.0;
    parameter Real attitudeMomentGain_N_m(unit = "N.m") = 0.65;
    parameter Real rateMomentGain_N_m_s(unit = "N.m.s") = 0.17;
    parameter Vehicles.Rdd2.RotorGeometry rotorGeometry =
      Vehicles.Rdd2.RotorGeometry();

    final parameter Real thrustCoefficient_N_s2(unit = "N.s2") =
      if hoverFundamentalTarget_Hz > 0.0 then
        vehicleMass_kg * gravity_m_s2
          / (4.0 * (2.0 * pi * hoverFundamentalTarget_Hz) ^ 2)
      else propellerThrustCoefficient_N_s2
      "Propeller sized either from the airframe or from the target tone";
    final parameter Real trimRotorSpeed_rad_s(unit = "rad/s") =
      sqrt(vehicleMass_kg * gravity_m_s2 / (4.0 * thrustCoefficient_N_s2))
      "Rotor speed that trims four rotors against weight";
    final parameter Real maximumRotorSpeed_rad_s(unit = "rad/s") =
      trimRotorSpeed_rad_s * sqrt(thrustToWeightRatio)
      "Full-command rotor speed that realises the commanded thrust margin";

    Vehicles.Rdd2.Plant plant(
      rotorGeometry = rotorGeometry,
      vehicleMass_kg = vehicleMass_kg,
      gravity_m_s2 = gravity_m_s2,
      omegaMax = maximumRotorSpeed_rad_s,
      initialGroundClearance_m = initialGroundClearance_m,
      thrustCoefficient_N_s2 = thrustCoefficient_N_s2,
      enableRotorVibration = vibrationEnabled,
      rotorVibrationAngular_rad_s = vibrationAngularAmplitude_rad_s,
      rotorVibrationSpecificForce_m_s2 = vibrationSpecificForceAmplitude_m_s2,
      enableImuAntiAliasFilter = antiAliasFilter);

    final parameter Real wrenchToRotorThrust[4, 4] =
      Control.Multirotor.Allocation.quadrotorWrenchToThrust(
        rotorGeometry.positionBodyFlu_m,
        rotorGeometry.yawMomentPerThrust_m);
    final parameter Real hoverFundamental_Hz(unit = "Hz") =
      plant.hoverRotorFrequency_Hz
      "Fundamental rotor tone derived from this case's propeller";
    final parameter Real hoverBladePass_Hz(unit = "Hz") =
      plant.hoverBladePassFrequency_Hz "Blade-pass tone at hover trim";

    output Real zohAttitudeError_rad(unit = "rad", start = 0.0, fixed = true)
      "Zero-order-hold composition error against the vibration-free truth";
    output Real fohAttitudeError_rad(unit = "rad", start = 0.0, fixed = true)
      "First-order-hold composition error against the vibration-free truth";
    output Real zohVelocityError_m_s(unit = "m/s", start = 0.0,
      fixed = true)
      "Zero-order-hold delta-velocity error against the vibration-free truth";
    output Real fohVelocityError_m_s(unit = "m/s", start = 0.0,
      fixed = true)
      "First-order-hold delta-velocity error against the vibration-free truth";
    output Real sensedAngularVibration_rad_s(unit = "rad/s")
      "Magnitude of the angular disturbance injected ahead of the sensor
       bandwidth limit";
    output Real sensedSpecificForceVibration_m_s2(unit = "m/s2")
      "Magnitude of the specific-force disturbance injected ahead of the
       sensor bandwidth limit";
    output Real altitudeError_m(unit = "m")
      "Hover hold error, a witness that the case really flew";

  protected
    Real thrustCommand_N(unit = "N");
    Real momentCommand_N_m[3](each unit = "N.m");
    Real referenceVelocity_m_s[3](each unit = "m/s", each start = 0.0,
      each fixed = true)
      "Specific force integrated on the vibration-free truth attitude";
    discrete Real zohDeltaPosition_m[3](each start = 0.0, each fixed = true);
    discrete Real zohDeltaVelocity_m_s[3](each start = 0.0,
      each fixed = true);
    discrete Real zohDeltaQuaternion[4](start = {1.0, 0.0, 0.0, 0.0},
      each fixed = true);
    discrete Real fohDeltaPosition_m[3](each start = 0.0, each fixed = true);
    discrete Real fohDeltaVelocity_m_s[3](each start = 0.0,
      each fixed = true);
    discrete Real fohDeltaQuaternion[4](start = {1.0, 0.0, 0.0, 0.0},
      each fixed = true);
    discrete Real previousAngularVelocitySample_rad_s[3](each start = 0.0,
      each fixed = true);
    discrete Real previousSpecificForceSample_m_s2[3](each start = 0.0,
      each fixed = true);
    discrete Real scratchRotationJacobian[3, 3](each start = 0.0,
      each fixed = true);
    discrete Real scratchVelocityGyroscopeJacobian[3, 3](each start = 0.0,
      each fixed = true);
    discrete Real scratchVelocityAccelerometerJacobian[3, 3](
      each start = 0.0, each fixed = true);
    discrete Real scratchPositionGyroscopeJacobian[3, 3](each start = 0.0,
      each fixed = true);
    discrete Real scratchPositionAccelerometerJacobian[3, 3](
      each start = 0.0, each fixed = true);

  equation
    // Truth-feedback hover hold. The controller reads the vibration-free
    // truth on purpose, so the study measures the inertial mechanization
    // cost of vibration without also folding in a control response to it.
    thrustCommand_N = max(0.0, plant.vehicleMass_kg * (plant.gravity_m_s2
      + altitudeGain_1_s2
        * (hoverAltitude_m - plant.truth.positionWorldEnu_m[3])
      - climbRateGain_1_s * plant.truth.velocityWorldEnu_m_s[3]));
    momentCommand_N_m = -attitudeMomentGain_N_m * plant.truth.eulerRpy_rad
      - rateMomentGain_N_m_s * plant.truth.angularVelocityBodyFlu_rad_s;
    plant.commands.motor = Control.Multirotor.Allocation.rotorCommands(
      4,
      thrustCommand_N,
      momentCommand_N_m,
      wrenchToRotorThrust,
      fill(thrustCoefficient_N_s2, 4),
      fill(maximumRotorSpeed_rad_s, 4));

    der(referenceVelocity_m_s) = plant.truth.accelerationWorldEnu_m_s2
      + {0.0, 0.0, plant.gravity_m_s2};
    altitudeError_m = plant.truth.positionWorldEnu_m[3] - hoverAltitude_m;
    sensedAngularVibration_rad_s = MathUtilities.norm3(
      plant.rotorVibrationAngularVelocityBodyFlu_rad_s);
    sensedSpecificForceVibration_m_s2 = MathUtilities.norm3(
      plant.rotorVibrationSpecificForceBodyFlu_m_s2);

  algorithm
    when sample(0.0, imuSamplePeriod_s) then
      if time >= 0.5 * imuSamplePeriod_s then
        // Mirror the vehicle driver: the sample closing the interval is held
        // over it under the zero-order hold, while the first-order hold
        // interpolates from the sample that opened it.
        (zohDeltaPosition_m,
         zohDeltaVelocity_m_s,
         zohDeltaQuaternion,
         scratchRotationJacobian,
         scratchVelocityGyroscopeJacobian,
         scratchVelocityAccelerometerJacobian,
         scratchPositionGyroscopeJacobian,
         scratchPositionAccelerometerJacobian) :=
          Estimation.StrapdownINS.preintegrateImuStep(
            pre(zohDeltaPosition_m),
            pre(zohDeltaVelocity_m_s),
            pre(zohDeltaQuaternion),
            zeros(3, 3), zeros(3, 3), zeros(3, 3), zeros(3, 3), zeros(3, 3),
            plant.imu.angularVelocityBodyFlu_rad_s,
            plant.imu.specificForceBodyFlu_m_s2,
            zeros(3), zeros(3), imuSamplePeriod_s);
        (fohDeltaPosition_m,
         fohDeltaVelocity_m_s,
         fohDeltaQuaternion,
         scratchRotationJacobian,
         scratchVelocityGyroscopeJacobian,
         scratchVelocityAccelerometerJacobian,
         scratchPositionGyroscopeJacobian,
         scratchPositionAccelerometerJacobian) :=
          Estimation.StrapdownINS.preintegrateImuStep(
            pre(fohDeltaPosition_m),
            pre(fohDeltaVelocity_m_s),
            pre(fohDeltaQuaternion),
            zeros(3, 3), zeros(3, 3), zeros(3, 3), zeros(3, 3), zeros(3, 3),
            plant.imu.angularVelocityBodyFlu_rad_s,
            plant.imu.specificForceBodyFlu_m_s2,
            zeros(3), zeros(3), imuSamplePeriod_s,
            true,
            pre(previousAngularVelocitySample_rad_s),
            pre(previousSpecificForceSample_m_s2));
      end if;
      previousAngularVelocitySample_rad_s :=
        plant.imu.angularVelocityBodyFlu_rad_s;
      previousSpecificForceSample_m_s2 :=
        plant.imu.specificForceBodyFlu_m_s2;
      zohAttitudeError_rad := MathUtilities.norm3(
        LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
          LieGroups.SO3.Quat.inverse(plant.truth.quaternionWorldBody),
          zohDeltaQuaternion)));
      fohAttitudeError_rad := MathUtilities.norm3(
        LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
          LieGroups.SO3.Quat.inverse(plant.truth.quaternionWorldBody),
          fohDeltaQuaternion)));
      zohVelocityError_m_s := MathUtilities.norm3(
        zohDeltaVelocity_m_s - referenceVelocity_m_s);
      fohVelocityError_m_s := MathUtilities.norm3(
        fohDeltaVelocity_m_s - referenceVelocity_m_s);
    end when;
  end HoverCase;

  // The same hover with no rotor content, which is the mechanization floor
  // that every vibration case has to be read against.
  HoverCase noVibration(vibrationEnabled = false);
  // Amplitude sweep on the RDD2 propeller: a well-isolated frame, the model
  // default, and a worn or unbalanced set.
  HoverCase mildAmplitude(
    vibrationAngularAmplitude_rad_s = 0.02,
    vibrationSpecificForceAmplitude_m_s2 = 0.4);
  HoverCase nominalAmplitude;
  HoverCase severeAmplitude(
    vibrationAngularAmplitude_rad_s = 0.15,
    vibrationSpecificForceAmplitude_m_s2 = 3.0);
  // The same nominal amplitude carried by a larger slower propeller and by a
  // smaller faster one, which moves the tone without moving the excitation.
  HoverCase slowPropeller(hoverFundamentalTarget_Hz = 80.0);
  HoverCase fastPropeller(hoverFundamentalTarget_Hz = 200.0);
  // The model default amplitude with the sensor bandwidth limit removed,
  // which is how the plant behaved before the anti-alias filter existed.
  HoverCase nominalWithoutAntiAlias(antiAliasFilter = false);

  output Real fundamentalTones_Hz[7];
  output Real bladePassTones_Hz[7];
  output Real zohAttitudeErrors_rad[7];
  output Real fohAttitudeErrors_rad[7];
  output Real zohVelocityErrors_m_s[7];
  output Real fohVelocityErrors_m_s[7];
  output Real sensedAngularVibration_rad_s[7];
  output Real sensedSpecificForceVibration_m_s2[7];
  output Real altitudeErrors_m[7];

equation
  fundamentalTones_Hz = {
    noVibration.hoverFundamental_Hz,
    mildAmplitude.hoverFundamental_Hz,
    nominalAmplitude.hoverFundamental_Hz,
    severeAmplitude.hoverFundamental_Hz,
    slowPropeller.hoverFundamental_Hz,
    fastPropeller.hoverFundamental_Hz,
    nominalWithoutAntiAlias.hoverFundamental_Hz};
  bladePassTones_Hz = {
    noVibration.hoverBladePass_Hz,
    mildAmplitude.hoverBladePass_Hz,
    nominalAmplitude.hoverBladePass_Hz,
    severeAmplitude.hoverBladePass_Hz,
    slowPropeller.hoverBladePass_Hz,
    fastPropeller.hoverBladePass_Hz,
    nominalWithoutAntiAlias.hoverBladePass_Hz};
  zohAttitudeErrors_rad = {
    noVibration.zohAttitudeError_rad,
    mildAmplitude.zohAttitudeError_rad,
    nominalAmplitude.zohAttitudeError_rad,
    severeAmplitude.zohAttitudeError_rad,
    slowPropeller.zohAttitudeError_rad,
    fastPropeller.zohAttitudeError_rad,
    nominalWithoutAntiAlias.zohAttitudeError_rad};
  fohAttitudeErrors_rad = {
    noVibration.fohAttitudeError_rad,
    mildAmplitude.fohAttitudeError_rad,
    nominalAmplitude.fohAttitudeError_rad,
    severeAmplitude.fohAttitudeError_rad,
    slowPropeller.fohAttitudeError_rad,
    fastPropeller.fohAttitudeError_rad,
    nominalWithoutAntiAlias.fohAttitudeError_rad};
  zohVelocityErrors_m_s = {
    noVibration.zohVelocityError_m_s,
    mildAmplitude.zohVelocityError_m_s,
    nominalAmplitude.zohVelocityError_m_s,
    severeAmplitude.zohVelocityError_m_s,
    slowPropeller.zohVelocityError_m_s,
    fastPropeller.zohVelocityError_m_s,
    nominalWithoutAntiAlias.zohVelocityError_m_s};
  fohVelocityErrors_m_s = {
    noVibration.fohVelocityError_m_s,
    mildAmplitude.fohVelocityError_m_s,
    nominalAmplitude.fohVelocityError_m_s,
    severeAmplitude.fohVelocityError_m_s,
    slowPropeller.fohVelocityError_m_s,
    fastPropeller.fohVelocityError_m_s,
    nominalWithoutAntiAlias.fohVelocityError_m_s};
  sensedAngularVibration_rad_s = {
    noVibration.sensedAngularVibration_rad_s,
    mildAmplitude.sensedAngularVibration_rad_s,
    nominalAmplitude.sensedAngularVibration_rad_s,
    severeAmplitude.sensedAngularVibration_rad_s,
    slowPropeller.sensedAngularVibration_rad_s,
    fastPropeller.sensedAngularVibration_rad_s,
    nominalWithoutAntiAlias.sensedAngularVibration_rad_s};
  sensedSpecificForceVibration_m_s2 = {
    noVibration.sensedSpecificForceVibration_m_s2,
    mildAmplitude.sensedSpecificForceVibration_m_s2,
    nominalAmplitude.sensedSpecificForceVibration_m_s2,
    severeAmplitude.sensedSpecificForceVibration_m_s2,
    slowPropeller.sensedSpecificForceVibration_m_s2,
    fastPropeller.sensedSpecificForceVibration_m_s2,
    nominalWithoutAntiAlias.sensedSpecificForceVibration_m_s2};
  altitudeErrors_m = {
    noVibration.altitudeError_m,
    mildAmplitude.altitudeError_m,
    nominalAmplitude.altitudeError_m,
    severeAmplitude.altitudeError_m,
    slowPropeller.altitudeError_m,
    fastPropeller.altitudeError_m,
    nominalWithoutAntiAlias.altitudeError_m};

  annotation(
    experiment(StartTime = 0.0, StopTime = 8.0, Tolerance = 1.0e-8,
      Interval = 0.005),
    Documentation(info = "<html>
    <p>Flies the RDD2 plant in a truth-feedback hover under
    <code>Vehicles.Rdd2.RotorVibration</code> and composes its 1 kHz IMU
    stream the way the vehicle does, so the reported attitude and velocity
    errors are the inertial mechanization cost of vibration alone.</p>
    <p>The first case carries no rotor content and is the mechanization
    floor. Every other column has to be read against it, because composing
    a 1 kHz stream over eight seconds carries its own error whether or not
    the airframe vibrates.</p>
    <p>Each case reports its own derived hover tone. The tone is never
    assumed: the propeller thrust constant fixes the rotor speed that trims
    four rotors against weight, and the fundamental follows from it. The two
    propeller cases invert that relation, sizing a propeller from a target
    tone, and hold the same thrust-to-weight margin so they keep the same
    control authority. Their excitation amplitude is unchanged because the
    amplitude law is referenced to each case own hover speed.</p>
    <p>The zero-order-hold column is the composition the vehicle uses today.
    The first-order-hold column is the same interval composed with the
    coning, sculling, and scrolling cross terms, so the gap between the two
    is what accumulation buys under this disturbance.</p>
    <p>The last case removes the sensor bandwidth limit, which is how the
    plant behaved before the ICM-45686 user-interface filter was modelled.
    Its difference from the nominal case is the amount by which unfiltered
    rotor content had been misrepresenting the hardware.</p>
  </html>"));
end RotorVibrationHoverStudy;
