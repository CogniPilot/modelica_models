within Control.Multirotor;
package LogLinear
  "Left-invariant SE_2(3) position and SO(3) attitude control"

  package Gains "Hover-linearized continuous-time LQR gains"
    constant Real positionHorizontal = -2.775042319292923;
    constant Real positionVertical = -2.786485597992051;
    constant Real positionVelocityHorizontal = 0.4285602828231471;
    constant Real positionVelocityVertical = 0.4852813742385701;
    constant Real positionAttitudeCoupling = 0.339817907297276;
    constant Real velocityHorizontal = -1.9507053321251042;
    constant Real velocityVertical = -2.9555142930282465;
    constant Real velocityAttitudeCoupling = 2.2064009963856357;
    constant Real tilt = -6.801601132247243;
    constant Real yaw = -2.82842712474619;

    constant Real feedback[9, 9] = [
      positionHorizontal, 0.0, 0.0,
        positionVelocityHorizontal, 0.0, 0.0,
        0.0, positionAttitudeCoupling, 0.0;
      0.0, positionHorizontal, 0.0,
        0.0, positionVelocityHorizontal, 0.0,
        -positionAttitudeCoupling, 0.0, 0.0;
      0.0, 0.0, positionVertical,
        0.0, 0.0, positionVelocityVertical,
        0.0, 0.0, 0.0;
      positionVelocityHorizontal, 0.0, 0.0,
        velocityHorizontal, 0.0, 0.0,
        0.0, -velocityAttitudeCoupling, 0.0;
      0.0, positionVelocityHorizontal, 0.0,
        0.0, velocityHorizontal, 0.0,
        velocityAttitudeCoupling, 0.0, 0.0;
      0.0, 0.0, positionVelocityVertical,
        0.0, 0.0, velocityVertical,
        0.0, 0.0, 0.0;
      0.0, -positionAttitudeCoupling, 0.0,
        0.0, velocityAttitudeCoupling, 0.0,
        tilt, 0.0, 0.0;
      positionAttitudeCoupling, 0.0, 0.0,
        -velocityAttitudeCoupling, 0.0, 0.0,
        0.0, tilt, 0.0;
      0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, yaw]
      "Gain K for the convention u = K zeta";

    annotation(Documentation(info="<html>
      <p>The default matrix is the continuous-time algebraic-Riccati solution
      about hover with <code>Q = 8 I</code>, <code>R = I</code>, and
      <code>g = 9.8 m/s2</code>. It is exposed as a replaceable controller
      parameter; applications with different dynamics or cost weights should
      supply their own gain.</p>
    </html>"));
  end Gains;

  record OuterLoopResult "Translational-control result"
    Real thrust(unit = "N") "Collective thrust force";
    Real attitudeSetpoint[4](each unit = "1")
      "Body-to-world quaternion {w,x,y,z}";
    Real angularVelocityCorrection[3](each unit = "rad/s")
      "Rotational part of the SE_2(3) feedback correction";
  end OuterLoopResult;

  function stateError
    "Compute the left-invariant SE_2(3) error log(actual^(-1) reference)"
    input Real positionWorld[3](each unit = "m") "Actual world position";
    input Real velocityWorld[3](each unit = "m/s") "Actual world velocity";
    input Real quaternionWorldBody[4](each unit = "1")
      "Actual body-to-world quaternion {w,x,y,z}";
    input Real positionReferenceWorld[3](each unit = "m")
      "Reference world position";
    input Real velocityReferenceWorld[3](each unit = "m/s")
      "Reference world velocity";
    input Real quaternionReferenceWorldBody[4](each unit = "1")
      "Reference body-to-world quaternion {w,x,y,z}";
    output Real error[9]
      "SE_2(3) tangent ordered {position,velocity,rotation}";
  protected
    Real actual[10];
    Real reference[10];
  algorithm
    actual := cat(1, positionWorld, velocityWorld, quaternionWorldBody);
    reference := cat(
      1,
      positionReferenceWorld,
      velocityReferenceWorld,
      quaternionReferenceWorldBody);
    error := LieGroups.SE23.Quat.log_map(
      LieGroups.SE23.Quat.product(
        LieGroups.SE23.Quat.inverse(actual),
        reference));
    annotation(Documentation(info="<html>
      <p>The actual and reference navigation states are represented as
      SE_2(3) elements <code>{position, velocity, quaternion}</code>. The
      logarithm of <code>actual^-1 reference</code> expresses the correction
      in actual-body coordinates.</p>
    </html>"));
  end stateError;

  function attitudeControl
    "Compute body angular velocity from an SO(3) logarithmic attitude error"
    input Real gain[3](each unit = "1/s") "Principal-axis attitude gains";
    input Real quaternionWorldBody[4](each unit = "1")
      "Actual body-to-world quaternion";
    input Real quaternionReferenceWorldBody[4](each unit = "1")
      "Reference body-to-world quaternion";
    output Real angularVelocityBody[3](each unit = "rad/s")
      "Commanded body angular velocity";
  protected
    Real error[3](each unit = "rad") "SO(3) logarithmic error";
  algorithm
    error := LieGroups.SO3.Quat.log_map(
      LieGroups.SO3.Quat.product(
        LieGroups.SO3.Quat.inverse(quaternionWorldBody),
        quaternionReferenceWorldBody));
    angularVelocityBody := LieGroups.SO3.Quat.left_jacobian(error)
      * diagonal(gain) * error;
    annotation(Documentation(info="<html>
      <p>Uses the shortest-path SO(3) logarithm and maps the proportional
      correction through the left Jacobian. Inputs must be unit,
      scalar-first Hamilton quaternions.</p>
    </html>"));
  end attitudeControl;

  function integratePositionError
    "Advance and symmetrically bound the position-error integral"
    input Real stateError[9]
      "SE_2(3) tangent ordered {position,velocity,rotation}";
    input Real positionIntegral[3] "Current integral state";
    input Real dt(unit = "s") "Integration interval";
    input Real thrustTrim(unit = "N") "Nominal collective thrust";
    input Real integralGain[3] = {0.1, 0.1, 0.1};
    input Real integralFractionLimit[3] = {0.25, 0.25, 0.25}
      "Maximum integral contribution as a fraction of trim thrust";
    output Real nextPositionIntegral[3] "Bounded next integral state";
  protected
    Real limit;
  algorithm
    assert(dt >= 0.0, "Controller time step must be nonnegative");
    assert(thrustTrim >= 0.0, "Trim thrust must be nonnegative");
    for axis in 1:3 loop
      assert(integralGain[axis] > 0.0,
        "Every position integral gain must be positive");
      assert(integralFractionLimit[axis] >= 0.0,
        "Every position integral limit must be nonnegative");
      limit := thrustTrim * integralFractionLimit[axis] / integralGain[axis];
      nextPositionIntegral[axis] := MathUtilities.clip(
        positionIntegral[axis] + dt * stateError[axis],
        -limit,
        limit);
    end for;
    annotation(Documentation(info="<html>
      <p>Applies a forward-Euler step to the translational part of the
      invariant error. Each axis is limited so that its gain-weighted
      contribution cannot exceed the configured fraction of trim thrust.</p>
    </html>"));
  end integratePositionError;

  function outerLoop
    "Construct collective thrust and attitude from SE_2(3) feedback"
    input Real stateError[9]
      "SE_2(3) tangent ordered {position,velocity,rotation}";
    input Real accelerationReferenceWorld[3](each unit = "m/s2")
      "Feed-forward world acceleration";
    input Real quaternionWorldBody[4](each unit = "1")
      "Actual body-to-world quaternion used to rotate invariant errors";
    input Real headingQuaternionWorldBody[4](each unit = "1")
      "Yaw reference used to construct the commanded body frame";
    input Real positionIntegral[3] "Position-error integral state";
    input Real thrustTrim(unit = "N") "Nominal collective thrust";
    input Real mass(unit = "kg") = 1.0 "Vehicle mass";
    input Real gravity(unit = "m/s2") = 9.80665
      "Positive gravitational acceleration";
    input Real integralGain[3] = {0.1, 0.1, 0.1};
    input Real translationalCorrectionFraction = 0.3
      "Maximum feedback force divided by vehicle weight";
    input Real feedbackGain[9, 9] =
      Control.Multirotor.LogLinear.Gains.feedback;
    output Control.Multirotor.LogLinear.OuterLoopResult result;
  protected
    constant Real worldX[3] = {1.0, 0.0, 0.0};
    constant Real worldY[3] = {0.0, 1.0, 0.0};
    constant Real worldZ[3] = {0.0, 0.0, 1.0};
    Real correction[9];
    Real translationalCorrectionWorld[3](each unit = "N");
    Real translationalCorrectionNorm(unit = "N");
    Real translationalCorrectionLimit(unit = "N");
    Real thrustVectorWorld[3](each unit = "N");
    Real thrustDirectionWorld[3];
    Real headingEuler[3](each unit = "rad");
    Real headingDirectionWorld[3];
    Real headingBasisWorld[3];
    Real bodyYWorld[3];
    Real bodyYNorm;
    Real bodyXWorld[3];
    Real bodyToWorld[3, 3];
  algorithm
    assert(mass > 0.0, "Vehicle mass must be positive");
    assert(gravity > 0.0, "Gravity must be positive");
    assert(thrustTrim >= 0.0, "Trim thrust must be nonnegative");
    assert(translationalCorrectionFraction >= 0.0,
      "The translational correction fraction must be nonnegative");

    correction := -LieGroups.SE23.Quat.left_jacobian(stateError)
      * feedbackGain * stateError;
    result.angularVelocityCorrection := correction[7:9];

    translationalCorrectionWorld :=
      LieGroups.SO3.Quat.rotate(quaternionWorldBody, correction[1:3])
      + LieGroups.SO3.Quat.rotate(
          quaternionWorldBody, correction[4:6])
      + mass * accelerationReferenceWorld;
    translationalCorrectionNorm :=
      MathUtilities.norm3(translationalCorrectionWorld);
    translationalCorrectionLimit :=
      translationalCorrectionFraction * mass * gravity;
    if translationalCorrectionNorm > translationalCorrectionLimit then
      translationalCorrectionWorld := translationalCorrectionLimit
        * translationalCorrectionWorld / translationalCorrectionNorm;
    end if;

    thrustVectorWorld := translationalCorrectionWorld + thrustTrim * worldZ
      + LieGroups.SO3.Quat.rotate(
          quaternionWorldBody, integralGain .* positionIntegral);
    result.thrust := MathUtilities.norm3(thrustVectorWorld);
    thrustDirectionWorld := if result.thrust > 1.0e-3 then
        thrustVectorWorld / result.thrust
      else
        worldZ;

    headingEuler := LieGroups.SO3.EulerB321.from_Quat(
      headingQuaternionWorldBody);
    headingDirectionWorld := {
      cos(headingEuler[1]),
      sin(headingEuler[1]),
      0.0};
    headingBasisWorld := headingDirectionWorld;
    bodyYWorld := cross(thrustDirectionWorld, headingBasisWorld);
    bodyYNorm := MathUtilities.norm3(bodyYWorld);
    if bodyYNorm <= 1.0e-3 then
      headingBasisWorld := if abs(thrustDirectionWorld[1])
          <= abs(thrustDirectionWorld[2])
          and abs(thrustDirectionWorld[1])
          <= abs(thrustDirectionWorld[3]) then
          worldX
        else if abs(thrustDirectionWorld[2])
          <= abs(thrustDirectionWorld[3]) then
          worldY
        else
          worldZ;
      bodyYWorld := cross(thrustDirectionWorld, headingBasisWorld);
      bodyYNorm := MathUtilities.norm3(bodyYWorld);
    end if;
    assert(bodyYNorm > 1.0e-6,
      "Unable to construct a body frame from the thrust direction");
    bodyYWorld := bodyYWorld / bodyYNorm;
    bodyXWorld := cross(bodyYWorld, thrustDirectionWorld);
    bodyToWorld[:, 1] := bodyXWorld;
    bodyToWorld[:, 2] := bodyYWorld;
    bodyToWorld[:, 3] := thrustDirectionWorld;
    result.attitudeSetpoint := LieGroups.SO3.Quat.from_DCM(bodyToWorld);
    annotation(Documentation(info="<html>
      <p>Applies the nonlinear left-Jacobian correction
      <code>-J_l(zeta) K zeta</code>, rotates its translational components by
      the actual body-to-world attitude, adds acceleration feed-forward and trim, and
      converts the resulting thrust vector into collective thrust and a
      body-to-world attitude setpoint.</p>
      <p>The feedback force magnitude is limited before trim and integral
      terms are added. The full body-frame integral correction is rotated to
      world coordinates. If the thrust vector is nearly zero, world z is used
      as the thrust direction; a least-aligned Cartesian axis resolves the
      heading singularity.</p>
    </html>"));
  end outerLoop;

  model Controller
    "Sampled integral state around the continuous log-linear control maps"
    parameter Real samplePeriod(unit = "s") = 0.01;
    parameter Real mass(unit = "kg") = 1.0;
    parameter Real gravity(unit = "m/s2") = 9.80665;
    parameter Real thrustTrim(unit = "N") = mass * gravity;
    parameter Real attitudeGain[3](each unit = "1/s") = {2.0, 2.0, 1.0};
    parameter Real integralGain[3] = {0.1, 0.1, 0.1};
    parameter Real integralFractionLimit[3] = {0.25, 0.25, 0.25};
    parameter Real translationalCorrectionFraction = 0.3;
    parameter Real feedbackGain[9, 9] =
      Control.Multirotor.LogLinear.Gains.feedback;

    input Real positionWorld[3](each unit = "m", each start = 0.0);
    input Real velocityWorld[3](each unit = "m/s", each start = 0.0);
    input Real quaternionWorldBody[4](
      each unit = "1",
      start = {1.0, 0.0, 0.0, 0.0});
    input Real positionReferenceWorld[3](each unit = "m", each start = 0.0);
    input Real velocityReferenceWorld[3](each unit = "m/s", each start = 0.0);
    input Real accelerationReferenceWorld[3](
      each unit = "m/s2",
      each start = 0.0);
    input Real headingQuaternionReference[4](
      each unit = "1",
      start = {1.0, 0.0, 0.0, 0.0});
    input Boolean resetIntegral(start = false);

    output Real stateError[9]
      "SE_2(3) tangent ordered {position,velocity,rotation}";
    output Real thrust(unit = "N") "Collective thrust force";
    output Real attitudeSetpoint[4](each unit = "1")
      "Body-to-world quaternion {w,x,y,z}";
    output Real angularVelocitySetpoint[3](each unit = "rad/s")
      "Commanded body angular velocity";
    output Real angularVelocityCorrection[3](each unit = "rad/s")
      "Rotational part of the SE_2(3) correction";
    discrete output Real positionIntegral[3](
      each start = 0.0,
      each fixed = true);

  protected
    Control.Multirotor.LogLinear.OuterLoopResult outerLoopResult;
  equation
    assert(samplePeriod > 0.0, "Controller sample period must be positive");
    stateError = Control.Multirotor.LogLinear.stateError(
      positionWorld,
      velocityWorld,
      quaternionWorldBody,
      positionReferenceWorld,
      velocityReferenceWorld,
      headingQuaternionReference);
    outerLoopResult = Control.Multirotor.LogLinear.outerLoop(
      stateError,
      accelerationReferenceWorld,
      quaternionWorldBody,
      headingQuaternionReference,
      positionIntegral,
      thrustTrim,
      mass,
      gravity,
      integralGain,
      translationalCorrectionFraction,
      feedbackGain);
    thrust = outerLoopResult.thrust;
    attitudeSetpoint = outerLoopResult.attitudeSetpoint;
    angularVelocityCorrection = outerLoopResult.angularVelocityCorrection;
    angularVelocitySetpoint = Control.Multirotor.LogLinear.attitudeControl(
      attitudeGain,
      quaternionWorldBody,
      attitudeSetpoint);

    when sample(0.0, samplePeriod) then
      positionIntegral = if resetIntegral then
          zeros(3)
        else
          Control.Multirotor.LogLinear.integratePositionError(
            stateError,
            pre(positionIntegral),
            samplePeriod,
            thrustTrim,
            integralGain,
            integralFractionLimit);
    end when;

    annotation(Documentation(info="<html>
      <p>Combines the pure control functions with a sampled, bounded position
      integral. The geometric proportional feedback remains algebraic and is
      therefore available continuously between integral updates.</p>
      <p>Vehicle packages normally extend this model to provide mass, gravity,
      trim thrust, and gains. Connect <code>angularVelocitySetpoint</code> to a
      body-rate loop and <code>thrust</code> to the vehicle's control allocator.</p>
    </html>"));
  end Controller;

  annotation(Documentation(info="<html>
    <h4>Overview</h4>
    <p>This package implements a coordinate-free multirotor outer loop. A
    navigation state is represented on SE_2(3); position, velocity, and
    attitude error are coupled through a left-invariant logarithm. A separate
    SO(3) logarithmic loop produces the body-rate command.</p>
    <h4>Conventions</h4>
    <ul>
      <li>World z points upward.</li>
      <li>Quaternions are scalar-first Hamilton quaternions mapping body to world.</li>
      <li>SE_2(3) tangents are ordered position, velocity, rotation.</li>
      <li>Collective thrust is a force in newtons; the control allocator is external.</li>
    </ul>
    <h4>Use with a flat trajectory</h4>
    <p>Connect position, velocity, and acceleration from
    <code>Planning.Bezier.MultirotorTrajectory</code>. Construct
    <code>headingQuaternionReference</code> from the trajectory yaw. The full
    flatness attitude may be used for feed-forward analysis, while this
    controller constructs its own feedback-corrected thrust attitude.</p>
    <p>The implementation follows
    <a href='https://github.com/CogniPilot/cyecca/blob/main/cyecca/models/rdd2_loglinear.py'>
    cyecca/models/rdd2_loglinear.py</a>.</p>
  </html>"));
end LogLinear;
