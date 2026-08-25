within Vehicles.Rdd2;

model RotorVibration
  "Narrowband rotor unbalance and blade-pass airframe vibration"
  constant Real pi = 2.0 * asin(1.0);

  parameter Boolean enable = false
    "Enable the rotor vibration disturbance; false reproduces the
     vibration-free plant exactly and leaves no continuous state behind"
    annotation(Evaluate = true);
  parameter Real bladeCount(min = 1.0) = 2.0
    "Blades per rotor, setting the blade-pass harmonic order. Real, not
     Integer: the plant that instantiates this model is exported through
     FMI 3 and that backend has no Integer scalar type."
    annotation(Evaluate = true);
  parameter Vehicles.Rdd2.RotorGeometry rotorGeometry =
    Vehicles.Rdd2.RotorGeometry()
    "Rotor stations used to place each unbalance moment on the airframe";
  parameter Real hoverRotorSpeed_rad_s(unit = "rad/s", min = 1.0e-6)
    "Rotor speed at hover trim, the reference for the omega squared law";
  parameter Real hoverAngularVibration_rad_s(unit = "rad/s") = 0.05
    "Per-rotor angular-rate amplitude of the fundamental at hover trim";
  parameter Real hoverSpecificForceVibration_m_s2(unit = "m/s2") = 1.0
    "Per-rotor lateral specific-force amplitude of the fundamental at hover";
  parameter Real bladePassAngularRatio = 0.6
    "Blade-pass angular amplitude relative to the fundamental";
  parameter Real bladePassSpecificForceRatio = 0.8
    "Blade-pass specific-force amplitude relative to the fundamental";
  parameter Real axialSpecificForceRatio = 1.0
    "Axial (thrust-ripple) specific-force amplitude relative to the lateral pair";
  parameter Real yawTorqueRippleRatio = 0.3
    "Yaw-axis angular amplitude relative to the lateral pair";
  parameter Real specificForceQuadraturePhase_rad(unit = "rad") = 0.5 * pi
    "Phase lead of the specific-force vibration over the angular vibration;
     a quarter turn is the canonical sculling driver";
  parameter Real unbalancePhase_rad[4](each unit = "rad") =
    {0.0, 0.5 * pi, pi, 1.5 * pi}
    "Unbalance phase of each rotor, spread uniformly by default because that
     is the maximum-coning arrangement for the quad-X arm geometry";
  parameter Real bladePassPhase_rad[4](each unit = "rad") = unbalancePhase_rad
    "Blade-passage phase offset of each rotor on the lateral and yaw
     channels; blade indexing is no more registered to the airframe than
     the unbalance mark is, so it takes the same spread by default";
  parameter Real unbalanceScale[4] = ones(4)
    "Per-rotor unbalance magnitude, a manufacturing tolerance in practice";
  parameter Real spinDirection[4] = {1.0, 1.0, -1.0, -1.0}
    "Rotor spin sense in command order, setting the torque-ripple yaw sign";

  final parameter Real rotorRadius_m[4](each unit = "m") = {
    sqrt(rotorGeometry.positionBodyFlu_m[rotorIndex, 1] ^ 2
       + rotorGeometry.positionBodyFlu_m[rotorIndex, 2] ^ 2)
    for rotorIndex in 1:4}
    "In-plane distance of each rotor station from the body origin";
  final parameter Real momentAxisBodyFlu[4, 3] = [
     rotorGeometry.positionBodyFlu_m[:, 2] ./ rotorRadius_m,
    -rotorGeometry.positionBodyFlu_m[:, 1] ./ rotorRadius_m,
     zeros(4)]
    "Unit body axis about which each rotor's out-of-plane unbalance force
     rocks the airframe, r cross z-hat normalised";
  final parameter Real hoverFundamental_Hz(unit = "Hz") =
    hoverRotorSpeed_rad_s / (2.0 * pi)
    "Fundamental rotor tone at hover trim";
  final parameter Real hoverBladePass_Hz(unit = "Hz") =
    bladeCount * hoverFundamental_Hz "Blade-pass tone at hover trim";

  input Real rotorSpeed_rad_s[4](each unit = "rad/s")
    "Instantaneous rotor speeds in command order";
  output Real angularVelocityBodyFlu_rad_s[3](each unit = "rad/s")
    "Angular-rate vibration added to the sensed body rate";
  output Real specificForceBodyFlu_m_s2[3](each unit = "m/s2")
    "Specific-force vibration added to the sensed body specific force";

protected
  Real rotorPhase_rad[4](each unit = "rad", each start = 0.0,
    each fixed = enable)
    "Integrated rotor phase, so the tone follows throttle. It is a state
     only while the disturbance is enabled, so a disabled instance leaves
     the initialisation problem exactly as it was";
  Real speedRatioSquared[4]
    "Unbalance force law, amplitude proportional to rotor speed squared";
  Real fundamentalPhase_rad[4](each unit = "rad");
  Real harmonicPhase_rad[4](each unit = "rad");
  Real lateralAngular[4] "Per-rotor lateral angular modal amplitude";
  Real yawAngular[4] "Per-rotor yaw torque-ripple amplitude";
  Real lateralSpecificForce[4] "Per-rotor lateral specific-force amplitude";
  Real axialSpecificForce[4] "Per-rotor axial thrust-ripple amplitude";

equation
  if enable then
    der(rotorPhase_rad) = rotorSpeed_rad_s;
  else
    rotorPhase_rad = {0.0, 0.0, 0.0, 0.0};
  end if;

  for rotorIndex in 1:4 loop
    speedRatioSquared[rotorIndex] = unbalanceScale[rotorIndex]
      * (rotorSpeed_rad_s[rotorIndex] / hoverRotorSpeed_rad_s) ^ 2;
    fundamentalPhase_rad[rotorIndex] = rotorPhase_rad[rotorIndex]
      + unbalancePhase_rad[rotorIndex];
    harmonicPhase_rad[rotorIndex] = bladeCount * rotorPhase_rad[rotorIndex]
      + bladePassPhase_rad[rotorIndex];
    lateralAngular[rotorIndex] = speedRatioSquared[rotorIndex]
      * (cos(fundamentalPhase_rad[rotorIndex])
       + bladePassAngularRatio * cos(harmonicPhase_rad[rotorIndex]));
    yawAngular[rotorIndex] = spinDirection[rotorIndex]
      * yawTorqueRippleRatio * speedRatioSquared[rotorIndex]
      * (sin(fundamentalPhase_rad[rotorIndex])
       + bladePassAngularRatio * sin(harmonicPhase_rad[rotorIndex]));
    lateralSpecificForce[rotorIndex] = speedRatioSquared[rotorIndex]
      * (cos(fundamentalPhase_rad[rotorIndex]
           + specificForceQuadraturePhase_rad)
       + bladePassSpecificForceRatio * cos(harmonicPhase_rad[rotorIndex]
           + specificForceQuadraturePhase_rad));
    // Blade passage against the fixed arm and airframe wake is common mode
    // across the rotors, so the axial thrust ripple keeps a shared phase
    // reference and survives the arm symmetry that nulls the lateral sum.
    axialSpecificForce[rotorIndex] = axialSpecificForceRatio
      * speedRatioSquared[rotorIndex]
      * (cos(fundamentalPhase_rad[rotorIndex])
       + bladePassSpecificForceRatio
         * cos(bladeCount * rotorPhase_rad[rotorIndex]));
  end for;

  if enable then
    angularVelocityBodyFlu_rad_s = hoverAngularVibration_rad_s
      * (transpose(momentAxisBodyFlu) * lateralAngular
       + {0.0, 0.0, sum(yawAngular)});
    specificForceBodyFlu_m_s2 = hoverSpecificForceVibration_m_s2
      * (transpose(momentAxisBodyFlu) * lateralSpecificForce
       + {0.0, 0.0, sum(axialSpecificForce)});
  else
    angularVelocityBodyFlu_rad_s = {0.0, 0.0, 0.0};
    specificForceBodyFlu_m_s2 = {0.0, 0.0, 0.0};
  end if;

  annotation(Documentation(info = "<html>
    <p>Rotor vibration sensed by an airframe-mounted inertial measurement
    unit. Each rotor carries its own integrated phase, so every tone tracks
    the instantaneous rotor speed and the disturbance grows and shrinks with
    throttle through a mission rather than sitting at a fixed frequency.</p>

    <h4>Physical content</h4>
    <p>A rotor mass unbalance produces a force that rotates with the disc and
    scales with the square of rotor speed. Its out-of-plane component acts at
    the rotor station, so it rocks the airframe about the body axis
    <code>r &times; z</code>. Those axes are the four arm diagonals of the
    quad-X geometry, and with the unbalance phases spread uniformly around
    the disc the two lateral body axes receive equal amplitudes a quarter
    turn apart. That is genuine coning rather than an asserted phase
    relationship: it falls out of the arm geometry.</p>
    <p>Blade passage adds a harmonic at <code>bladeCount</code> times the
    fundamental. On the lateral and yaw channels it carries the same spread
    phases as the unbalance, because blade indexing is no more registered to
    the airframe than the unbalance mark is. The axial thrust ripple instead
    keeps a common phase reference across the rotors, because blade passage
    against a fixed arm and its wake is driven by airframe azimuth rather
    than by rotor build tolerance; that is what keeps the axial tone alive
    where the arm symmetry cancels the lateral one, and it matches flight
    logs in which the blade-pass tone dominates the accelerometer. Rotor
    torque ripple drives the yaw axis with the sign of each rotor spin
    direction.</p>
    <p>With the default phases and a matched set of rotors the lateral pair
    is a circle of amplitude twice <code>hoverAngularVibration_rad_s</code>,
    the axial accelerometer tone is four times
    <code>hoverSpecificForceVibration_m_s2</code> scaled by the axial and
    blade-pass ratios, and the fundamental cancels in the axial channel.
    Unequal rotor speeds through a mission detune the four tones, so the sum
    beats rather than holding a perfect circle.</p>

    <h4>Where the disturbance is applied</h4>
    <p>This is frame vibration that the airframe experiences and the IMU
    therefore senses. It is added to the sensed body rate and specific force
    and deliberately not fed back into the rigid-body integration: the motion
    is a small oscillation about the flight trim with no net displacement, so
    the mean trajectory is unchanged, while the rectified coning, sculling,
    and scrolling content that the disturbance creates appears only when
    sampled increments are composed downstream. Truth therefore stays the
    smooth trim trajectory, which is the trajectory the vehicle really
    flies.</p>
    <p>A structural resonance between the airframe and the IMU mount would be
    a sensed-only artifact rather than frame motion. It is not modelled here;
    it can be approximated by raising the amplitudes without changing the
    truth signals, which is exactly what this model already does.</p>
  </html>"));
end RotorVibration;
