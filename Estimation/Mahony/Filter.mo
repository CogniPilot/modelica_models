within Estimation.Mahony;

block Filter
  "Explicit complementary attitude filter with gyro-bias state"

  parameter Real samplePeriod(unit = "s", min = 1.0e-9) = 0.005;
  parameter Real gravity_m_s2(unit = "m/s2") = 9.81;
  parameter Real proportionalGain(unit = "1") = 1.0
    "kP: innovation feedback into the rate driving the quaternion";
  parameter Real integralGain(unit = "1") = 0.3
    "kI: innovation feedback into the gyro-bias state";
  parameter Real accelerometerWeight(unit = "1") = 1.0
    "Weight of the gravity-direction innovation inside omega_mes";
  parameter Real magnetometerWeight(unit = "1") = 0.5
    "Weight of the heading innovation inside omega_mes";
  parameter Boolean useMagnetometer = false
    "Enable the magnetometer heading correction (yaw observability)";
  parameter Real accelerometerMinimumNorm_g(unit = "1") = 0.6
    "Reject accelerometer samples below this fraction of gravity";
  parameter Real accelerometerMaximumNorm_g(unit = "1") = 1.4
    "Reject accelerometer samples above this multiple of gravity";
  parameter Real magneticFieldWorldEnu_T[3] =
    {18.0e-6, 4.0e-6, -47.0e-6}
    "Local geomagnetic field reference used only for heading";
  parameter Real initialQuaternion[4] = {1.0, 0.0, 0.0, 0.0}
    "World-from-body attitude loaded at reset (normalized internally)";

  input Real gyro_rad_s[3](start = {0.0, 0.0, 0.0});
  input Real accel_m_s2[3](start = {0.0, 0.0, gravity_m_s2});
  input Real magneticFieldBodyFlu_T[3](start = {1.0e-5, 0.0, 0.0});
  input Boolean reset(start = false);

  discrete output Real quaternion[4](start = {1.0, 0.0, 0.0, 0.0},
    each fixed = true)
    "World-from-body unit quaternion {w,x,y,z}";
  discrete output Real gyroBias_rad_s[3](start = {0.0, 0.0, 0.0},
    each fixed = true)
    "Estimated gyroscope bias, subtracted from the measured rate";

protected
  final parameter Real initialQuaternionNorm = sqrt(
    initialQuaternion[1] * initialQuaternion[1]
      + initialQuaternion[2] * initialQuaternion[2]
      + initialQuaternion[3] * initialQuaternion[3]
      + initialQuaternion[4] * initialQuaternion[4]);
  final parameter Real initialQuaternionUnit[4] = initialQuaternion
    / max(initialQuaternionNorm, 1.0e-6);
  final parameter Real referenceHeadingNorm_T = sqrt(
    magneticFieldWorldEnu_T[1] * magneticFieldWorldEnu_T[1]
      + magneticFieldWorldEnu_T[2] * magneticFieldWorldEnu_T[2]);
  final parameter Real referenceHeadingEast = magneticFieldWorldEnu_T[1]
    / max(referenceHeadingNorm_T, 1.0e-12);
  final parameter Real referenceHeadingNorth = magneticFieldWorldEnu_T[2]
    / max(referenceHeadingNorm_T, 1.0e-12);

  discrete Real accelerometerNorm_m_s2(start = gravity_m_s2,
    fixed = true);
  discrete Boolean accelerometerValid(start = true, fixed = true);
  discrete Real gravityBody[3](start = {0.0, 0.0, 1.0},
    each fixed = true)
    "Estimated gravity direction in body axes, third row of R(pre(q))";
  discrete Real innovation_rad[3](start = {0.0, 0.0, 0.0},
    each fixed = true)
    "omega_mes: weighted sum of vector-product direction errors";
  discrete Real magneticWorldEast(start = 0.0, fixed = true);
  discrete Real magneticWorldNorth(start = 0.0, fixed = true);
  discrete Real magneticHorizontalNorm(start = 0.0, fixed = true);
  discrete Real headingError(start = 0.0, fixed = true);
  discrete Real correctedRate_rad_s[3](start = {0.0, 0.0, 0.0},
    each fixed = true);
  discrete Real unnormalizedQuaternion[4](start = {1.0, 0.0, 0.0, 0.0},
    each fixed = true);
  discrete Real quaternionNorm(start = 1.0, fixed = true);
  discrete Real quaternionNormInverse(start = 1.0, fixed = true);

equation
  when sample(0.0, samplePeriod) then
    // Estimated body-frame gravity direction from the prior attitude:
    // the third row of the body-to-world rotation R(pre(quaternion)).
    gravityBody = {
      2.0 * (pre(quaternion[2]) * pre(quaternion[4])
        - pre(quaternion[1]) * pre(quaternion[3])),
      2.0 * (pre(quaternion[1]) * pre(quaternion[2])
        + pre(quaternion[3]) * pre(quaternion[4])),
      pre(quaternion[1]) * pre(quaternion[1])
        - pre(quaternion[2]) * pre(quaternion[2])
        - pre(quaternion[3]) * pre(quaternion[3])
        + pre(quaternion[4]) * pre(quaternion[4])};

    // Accelerometer innovation: measured direction cross estimated
    // direction (omega_mes of the explicit filter), gated on the sample
    // magnitude staying near one gravity.
    accelerometerNorm_m_s2 = sqrt(
      accel_m_s2[1] * accel_m_s2[1]
        + accel_m_s2[2] * accel_m_s2[2]
        + accel_m_s2[3] * accel_m_s2[3]);
    accelerometerValid =
      accelerometerNorm_m_s2
          >= accelerometerMinimumNorm_g * gravity_m_s2
        and accelerometerNorm_m_s2
          <= accelerometerMaximumNorm_g * gravity_m_s2;

    // Optional heading correction: rotate the measured field to world
    // axes with the prior attitude, drop the vertical component, and
    // feed the horizontal direction error back about the body-frame
    // vertical (the gravityBody direction).
    magneticWorldEast = if useMagnetometer then
      (1.0 - 2.0 * (pre(quaternion[3]) * pre(quaternion[3])
        + pre(quaternion[4]) * pre(quaternion[4])))
        * magneticFieldBodyFlu_T[1]
      + 2.0 * (pre(quaternion[2]) * pre(quaternion[3])
        - pre(quaternion[1]) * pre(quaternion[4]))
        * magneticFieldBodyFlu_T[2]
      + 2.0 * (pre(quaternion[2]) * pre(quaternion[4])
        + pre(quaternion[1]) * pre(quaternion[3]))
        * magneticFieldBodyFlu_T[3]
      else 0.0;
    magneticWorldNorth = if useMagnetometer then
      2.0 * (pre(quaternion[2]) * pre(quaternion[3])
        + pre(quaternion[1]) * pre(quaternion[4]))
        * magneticFieldBodyFlu_T[1]
      + (1.0 - 2.0 * (pre(quaternion[2]) * pre(quaternion[2])
        + pre(quaternion[4]) * pre(quaternion[4])))
        * magneticFieldBodyFlu_T[2]
      + 2.0 * (pre(quaternion[3]) * pre(quaternion[4])
        - pre(quaternion[1]) * pre(quaternion[2]))
        * magneticFieldBodyFlu_T[3]
      else 0.0;
    magneticHorizontalNorm = sqrt(
      magneticWorldEast * magneticWorldEast
        + magneticWorldNorth * magneticWorldNorth);
    headingError = if useMagnetometer
        and magneticHorizontalNorm
          > 0.05 * referenceHeadingNorm_T then
      (magneticWorldEast * referenceHeadingNorth
        - magneticWorldNorth * referenceHeadingEast)
        / magneticHorizontalNorm
      else 0.0;

    // omega_mes of the explicit complementary filter: each vectorial
    // measurement contributes (measured direction) x (estimated
    // direction); a gated-out sample contributes nothing, so the tick
    // falls back to pure gyro integration.
    innovation_rad = if reset then {0.0, 0.0, 0.0}
      else (if accelerometerValid then (accelerometerWeight
        / max(accelerometerNorm_m_s2, 1.0e-6)) * {
          accel_m_s2[2] * gravityBody[3]
            - accel_m_s2[3] * gravityBody[2],
          accel_m_s2[3] * gravityBody[1]
            - accel_m_s2[1] * gravityBody[3],
          accel_m_s2[1] * gravityBody[2]
            - accel_m_s2[2] * gravityBody[1]}
        else {0.0, 0.0, 0.0})
        + (magnetometerWeight * headingError) * gravityBody;

    // Bias dynamics of the explicit filter, forward-Euler discretized:
    // b_dot = -kI * omega_mes.
    gyroBias_rad_s = if reset then {0.0, 0.0, 0.0}
      else pre(gyroBias_rad_s)
        - (integralGain * samplePeriod) * innovation_rad;

    // Rate driving the quaternion: omega_y - b_hat + kP * omega_mes.
    correctedRate_rad_s = gyro_rad_s - gyroBias_rad_s
      + proportionalGain * innovation_rad;

    // One forward-Euler step of q_dot = 0.5 * q x (0, omega), followed
    // by renormalization back onto the unit sphere.
    unnormalizedQuaternion = {
      pre(quaternion[1]) + 0.5 * samplePeriod
        * (-pre(quaternion[2]) * correctedRate_rad_s[1]
          - pre(quaternion[3]) * correctedRate_rad_s[2]
          - pre(quaternion[4]) * correctedRate_rad_s[3]),
      pre(quaternion[2]) + 0.5 * samplePeriod
        * (pre(quaternion[1]) * correctedRate_rad_s[1]
          - pre(quaternion[4]) * correctedRate_rad_s[2]
          + pre(quaternion[3]) * correctedRate_rad_s[3]),
      pre(quaternion[3]) + 0.5 * samplePeriod
        * (pre(quaternion[4]) * correctedRate_rad_s[1]
          + pre(quaternion[1]) * correctedRate_rad_s[2]
          - pre(quaternion[2]) * correctedRate_rad_s[3]),
      pre(quaternion[4]) + 0.5 * samplePeriod
        * (-pre(quaternion[3]) * correctedRate_rad_s[1]
          + pre(quaternion[2]) * correctedRate_rad_s[2]
          + pre(quaternion[1]) * correctedRate_rad_s[3])};
    quaternionNorm = sqrt(
      unnormalizedQuaternion[1] * unnormalizedQuaternion[1]
        + unnormalizedQuaternion[2] * unnormalizedQuaternion[2]
        + unnormalizedQuaternion[3] * unnormalizedQuaternion[3]
        + unnormalizedQuaternion[4] * unnormalizedQuaternion[4]);
    quaternionNormInverse = 1.0 / max(quaternionNorm, 1.0e-3);
    quaternion = if reset then initialQuaternionUnit
      elseif quaternionNorm > 1.0e-3 then
        quaternionNormInverse * unnormalizedQuaternion
      else {1.0, 0.0, 0.0, 0.0};
  end when;

  annotation(Documentation(info = "<html>
    <p>Discrete-time explicit complementary filter after R. Mahony,
    T. Hamel, and J.-M. Pflimlin, &quot;Nonlinear Complementary Filters on
    the Special Orthogonal Group&quot;, IEEE Transactions on Automatic
    Control, vol. 53, no. 5, pp. 1203-1218, 2008 (the explicit
    complementary filter with bias correction, quaternion form). This is a
    clean-room implementation from the published mathematics; no
    third-party filter source code was consulted. Per tick of length
    <code>h = samplePeriod</code>:</p>
    <ul>
    <li><code>omega_mes = k_a * (v_a x vhat_a) + k_m * e_heading *
    vhat_g</code>, where <code>v_a</code> is the unit accelerometer sample,
    <code>vhat_a = vhat_g = R(q)' e3</code> is the estimated gravity
    direction, and <code>e_heading</code> is the horizontal-plane vector
    product of the world-rotated magnetometer sample with the reference
    field direction.</li>
    <li><code>b(+) = b - kI * omega_mes * h</code> (bias state).</li>
    <li><code>q(+) = normalize(q + 0.5 * h * q x (0, omega_y - b(+) + kP *
    omega_mes))</code> (quaternion state).</li>
    </ul>
    <p>Conventions: the quaternion is Hamilton, scalar-first {w,x,y,z},
    and rotates body vectors into the world frame (world-from-body), with
    ENU world axes and FLU body axes, matching
    <code>LieGroups.SO3.Quat</code>. The accelerometer measures specific
    force, so at rest it points along body-frame up.</p>
    <p>Discretization: one forward-Euler step plus renormalization per
    tick instead of the closed-form axis-angle exponential. The direction
    error of that choice scales as <code>(|omega| h)^3</code> per tick;
    at 2000 deg/s and 200 Hz it stays below 0.4 mrad per tick and far
    below the vectorial-correction authority, while avoiding a
    sine/cosine pair that dominates the budget on softfloat cores. The
    renormalization multiplies by one reciprocal instead of dividing per
    component, so a tick costs exactly two square roots (accelerometer
    norm and quaternion norm, three with the magnetometer) and at most
    three divides.</p>
    <p>Guards: the accelerometer innovation is gated on the sample
    magnitude staying inside a configurable band around one gravity; the
    heading innovation is gated on the world-horizontal field component
    exceeding 5 percent of the reference horizontal magnitude (a
    unit-consistent threshold in Tesla), rejecting near-vertical field
    geometry; the quaternion renormalization falls back to identity below
    norm 1.0e-3. A rejected sample contributes zero innovation, so the
    tick falls back to pure gyro integration. Everything is scalar
    arithmetic (cross and quaternion products written out), fits the
    <code>when sample(0, samplePeriod)</code> clocked equation shape the
    embedded-c-galec target accepts, and stays well conditioned in
    binary32.</p>
    <p>The convergence and vibration behavior is exercised by
    <code>Tests.MahonyFilterStudy</code>. The paper demonstrates
    almost-global convergence of the continuous filter; the study checks
    the discrete implementation recovers from a 170 degree initial error
    and holds steady-state accuracy under coning vibration.</p>
    <p>The measured embedded footprint through <code>rumoca compile
    --target embedded-c-galec</code> plus arm-none-eabi-gcc -Os is
    tabulated in the <code>Estimation.Mahony</code> package
    documentation: 280 B of state and about 1.8 to 2.6 KB of code, with
    354 instructions per accel-only update on a Cortex-M4F and 6187 on a
    softfloat Cortex-M3, well inside a 20 MHz, 320 KB part at 200 Hz to
    1 kHz update rates.</p>
  </html>"));
end Filter;
