within Vehicles.Rdd2;

model Plant "RDD2 quadrotor with native avionics-facing connectors"
  constant Real pi = 2.0 * asin(1.0);
  parameter Real omegaMax(unit = "rad/s") = 1100.0;
  parameter Real vehicleMass_kg(unit = "kg") = 2.0;
  parameter Real gravity_m_s2(unit = "m/s2") = 9.80665;
  parameter Real thrustCoefficient_N_s2(unit = "N.s2") = 8.54858e-6
    "Rotor thrust per squared rotor speed";
  parameter Real initialGroundClearance_m(unit = "m") = 0.02
    "Landing-leg clearance above the ground plane at initialisation; raising
     it starts a study in the air instead of on its legs";
  parameter Vehicles.Rdd2.RotorGeometry rotorGeometry =
    Vehicles.Rdd2.RotorGeometry();
  parameter Real rotorEffectiveness[4, 4] =
    Control.Multirotor.Allocation.rotorEffectiveness(
      rotorGeometry.positionBodyFlu_m,
      rotorGeometry.yawMomentPerThrust_m);

  parameter Boolean enableRotorVibration = false
    "Add rotor unbalance and blade-pass vibration to the sensed IMU signals;
     false reproduces the vibration-free plant exactly"
    annotation(Evaluate = true);
  parameter Real bladeCount(min = 1.0) = 2.0
    "Blades per rotor. Real, not Integer: this model is exported through
     FMI 3 for the board-in-the-loop plant and that backend has no Integer
     scalar type. The value only ever scales a frequency or a phase, and it
     is a whole number by construction."
    annotation(Evaluate = true);
  parameter Real rotorVibrationAngular_rad_s(unit = "rad/s") = 0.05
    "Per-rotor angular-rate vibration amplitude at hover trim";
  parameter Real rotorVibrationSpecificForce_m_s2(unit = "m/s2") = 1.0
    "Per-rotor lateral specific-force vibration amplitude at hover trim";

  parameter Boolean enableImuAntiAliasFilter = enableRotorVibration
    "Run the sensor low-pass that precedes sampling in the real part. The
     default follows the vibration switch so a vibration-free run keeps the
     historical bit-identical trace, while any run that carries rotor content
     gets the hardware bandwidth"
    annotation(Evaluate = true);
  parameter Real imuAntiAliasBandwidth_Hz(unit = "Hz") = 200.0
    "ICM-45686 user-interface filter bandwidth, the 1600 Hz output data rate
     divided by eight";
  parameter Real imuAntiAliasDamping = 0.5 * sqrt(2.0)
    "Second-order section damping; the default is maximally flat";

  final parameter Real hoverRotorSpeed_rad_s(unit = "rad/s") =
    sqrt(vehicleMass_kg * gravity_m_s2 / (4.0 * thrustCoefficient_N_s2))
    "Rotor speed that trims four-rotor thrust against weight";
  final parameter Real hoverRotorFrequency_Hz(unit = "Hz") =
    hoverRotorSpeed_rad_s / (2.0 * pi) "Fundamental rotor tone at hover";
  final parameter Real hoverBladePassFrequency_Hz(unit = "Hz") =
    bladeCount * hoverRotorFrequency_Hz "Blade-pass tone at hover";
  final parameter Real imuAntiAliasNatural_rad_s(unit = "rad/s") =
    2.0 * pi * imuAntiAliasBandwidth_Hz;

  Vehicles.Rdd2.ControllerInterfaces.MotorCommandInput commands;
  Avionics.ImuSampleOutput imu;
  Avionics.NavigationEstimateOutput truth;
  output Real motorOmega_rad_s[4];
  output Real rotorVibrationAngularVelocityBodyFlu_rad_s[3](
    each unit = "rad/s") "Angular-rate vibration added to the sensed rate";
  output Real rotorVibrationSpecificForceBodyFlu_m_s2[3](
    each unit = "m/s2")
    "Specific-force vibration added to the sensed specific force";

protected
  Vehicles.Templates.QuadrotorPlant dynamics(
    vehicle_mass = vehicleMass_kg,
    gravity = gravity_m_s2,
    vehicle_ixx = 0.02166666666666667,
    vehicle_iyy = 0.02166666666666667,
    vehicle_izz = 0.04000000000000001,
    arm_length = rotorGeometry.armLength_m,
    initial_ground_clearance = initialGroundClearance_m,
    Ct = thrustCoefficient_N_s2,
    Cm = rotorGeometry.rotorTorqueRatio_m,
    motor_moment_map = rotorEffectiveness[2:4, :]
  );
  Vehicles.Rdd2.RotorVibration vibration(
    enable = enableRotorVibration,
    bladeCount = bladeCount,
    rotorGeometry = rotorGeometry,
    hoverRotorSpeed_rad_s = hoverRotorSpeed_rad_s,
    hoverAngularVibration_rad_s = rotorVibrationAngular_rad_s,
    hoverSpecificForceVibration_m_s2 = rotorVibrationSpecificForce_m_s2);
  Real eulerB321_rad[3] "Lie-group B321 coordinates [yaw, pitch, roll]";
  Real sensedAngularVelocityBodyFlu_rad_s[3]
    "Body rate the sensor sees before its own low-pass";
  Real sensedSpecificForceBodyFlu_m_s2[3]
    "Specific force the sensor sees before its own low-pass";
  Real filteredAngularVelocityBodyFlu_rad_s[3](each start = 0.0,
    each fixed = enableImuAntiAliasFilter)
    "Anti-alias filter output on the gyroscope channel";
  Real filteredSpecificForceBodyFlu_m_s2[3](each start = 0.0,
    each fixed = enableImuAntiAliasFilter)
    "Anti-alias filter output on the accelerometer channel";
  Real filteredAngularVelocityRate_rad_s2[3](each start = 0.0,
    each fixed = enableImuAntiAliasFilter)
    "Gyroscope filter section velocity state";
  Real filteredSpecificForceRate_m_s3[3](each start = 0.0,
    each fixed = enableImuAntiAliasFilter)
    "Accelerometer filter section velocity state";

equation
  dynamics.omega_cmd = omegaMax * {
    MathUtilities.clip(commands.motor[motorIndex], 0.0, 1.0)
      for motorIndex in 1:4};

  vibration.rotorSpeed_rad_s = dynamics.omega_m;
  rotorVibrationAngularVelocityBodyFlu_rad_s =
    vibration.angularVelocityBodyFlu_rad_s;
  rotorVibrationSpecificForceBodyFlu_m_s2 =
    vibration.specificForceBodyFlu_m_s2;

  // The airframe experiences the rotor vibration and the sensor experiences
  // it with the airframe, so it enters the sensed signals ahead of the
  // sensor's own bandwidth limit and never enters the rigid-body integration.
  sensedAngularVelocityBodyFlu_rad_s =
    dynamics.gyro + vibration.angularVelocityBodyFlu_rad_s;
  sensedSpecificForceBodyFlu_m_s2 =
    dynamics.accel + vibration.specificForceBodyFlu_m_s2;

  if enableImuAntiAliasFilter then
    der(filteredAngularVelocityBodyFlu_rad_s) =
      filteredAngularVelocityRate_rad_s2;
    der(filteredAngularVelocityRate_rad_s2) =
      imuAntiAliasNatural_rad_s * imuAntiAliasNatural_rad_s
        * (sensedAngularVelocityBodyFlu_rad_s
         - filteredAngularVelocityBodyFlu_rad_s)
      - 2.0 * imuAntiAliasDamping * imuAntiAliasNatural_rad_s
        * filteredAngularVelocityRate_rad_s2;
    der(filteredSpecificForceBodyFlu_m_s2) = filteredSpecificForceRate_m_s3;
    der(filteredSpecificForceRate_m_s3) =
      imuAntiAliasNatural_rad_s * imuAntiAliasNatural_rad_s
        * (sensedSpecificForceBodyFlu_m_s2
         - filteredSpecificForceBodyFlu_m_s2)
      - 2.0 * imuAntiAliasDamping * imuAntiAliasNatural_rad_s
        * filteredSpecificForceRate_m_s3;
  else
    filteredAngularVelocityBodyFlu_rad_s =
      sensedAngularVelocityBodyFlu_rad_s;
    filteredAngularVelocityRate_rad_s2 = {0.0, 0.0, 0.0};
    filteredSpecificForceBodyFlu_m_s2 = sensedSpecificForceBodyFlu_m_s2;
    filteredSpecificForceRate_m_s3 = {0.0, 0.0, 0.0};
  end if;

  imu.valid = true;
  imu.fresh = true;
  imu.timestamp_s = time;
  imu.angularVelocityBodyFlu_rad_s = filteredAngularVelocityBodyFlu_rad_s;
  imu.specificForceBodyFlu_m_s2 = filteredSpecificForceBodyFlu_m_s2;
  // Literal arrays rather than zeros(): the board-in-the-loop plant is
  // exported through FMI 3, whose backend renders a tensor equation one
  // scalar at a time and has no scalar form for a fill node. The plant does
  // not preintegrate, so its raw sample carries the identity preintegral.
  imu.deltaAngleBodyFlu_rad = {0.0, 0.0, 0.0};
  imu.deltaVelocityBodyFlu_m_s = {0.0, 0.0, 0.0};
  imu.deltaPositionBodyFlu_m = {0.0, 0.0, 0.0};
  imu.deltaQuaternionBodyFlu = {1.0, 0.0, 0.0, 0.0};
  imu.integrationTime_s = 0.0;
  imu.gyroscopeBiasLinearizationBodyFlu_rad_s = {0.0, 0.0, 0.0};
  imu.accelerometerBiasLinearizationBodyFlu_m_s2 = {0.0, 0.0, 0.0};
  imu.deltaRotationGyroscopeBiasJacobian_s =
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  imu.deltaVelocityGyroscopeBiasJacobian_m =
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  imu.deltaVelocityAccelerometerBiasJacobian_s =
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  imu.deltaPositionGyroscopeBiasJacobian_m_s =
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  imu.deltaPositionAccelerometerBiasJacobian_s2 =
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

  eulerB321_rad = LieGroups.SO3.EulerB321.from_Quat(dynamics.quat);
  truth.valid = true;
  truth.timestamp_s = time;
  truth.positionWorldEnu_m = dynamics.position;
  truth.velocityWorldEnu_m_s = dynamics.velocity;
  truth.accelerationWorldEnu_m_s2 =
    dynamics.R * dynamics.accel + {0.0, 0.0, -dynamics.gravity};
  truth.quaternionWorldBody = dynamics.quat;
  truth.rotationWorldBody = dynamics.R;
  truth.eulerRpy_rad = {
    eulerB321_rad[3], eulerB321_rad[2], eulerB321_rad[1]};
  truth.angularVelocityBodyFlu_rad_s = dynamics.gyro;
  truth.angularVelocityWorldEnu_rad_s = dynamics.R * dynamics.gyro;
  motorOmega_rad_s = dynamics.omega_m;

  annotation(Documentation(info = "<html>
    <p>RDD2 quadrotor plant behind the avionics-facing IMU and truth
    connectors.</p>

    <h4>Rotor vibration</h4>
    <p><code>enableRotorVibration</code> adds
    <code>Vehicles.Rdd2.RotorVibration</code> to the sensed IMU signals. The
    hover reference frequency is derived here rather than assumed: trimming
    four rotors against weight gives
    <code>hoverRotorSpeed_rad_s = sqrt(mass * gravity / (4 * Ct))</code>,
    which for the RDD2 mass, gravity, and thrust coefficient puts the
    fundamental near 120 Hz and the two-blade passage near 241 Hz. The
    disturbance is driven from the live rotor speeds, so it tracks throttle
    through a mission. Truth outputs stay vibration free because the motion
    is an oscillation about the flight trim with no net displacement.</p>

    <h4>Sensor bandwidth</h4>
    <p><code>enableImuAntiAliasFilter</code> applies the second-order
    low-pass that the ICM-45686 runs ahead of its sampler. The part is
    configured at a 1600 Hz output data rate with the user-interface filter
    at one eighth of that, so the default bandwidth is 200 Hz. Without this
    filter the model hands unattenuated rotor content to a point sampler and
    misrepresents what the hardware delivers. The switch defaults to the
    vibration switch so vibration-free runs keep their historical trace, and
    it can be forced either way to separate the filter's contribution from
    the vibration's.</p>
  </html>"));
end Plant;
