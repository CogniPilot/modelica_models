within Control.Multirotor;
package RateLoop "Multirotor body-rate control"

  function bodyMoment
    "Body moment from a proportional body-rate error with gyroscopic feed-forward"
    input Real angularVelocitySetpoint[3](each unit = "rad/s")
      "Commanded body angular velocity";
    input Real angularVelocity[3](each unit = "rad/s")
      "Measured body angular velocity";
    input Real inertia[3](each unit = "kg.m2")
      "Diagonal body inertia {ixx, iyy, izz}";
    input Real rateGain[3](each unit = "1/s")
      "Proportional body-rate bandwidth per axis";
    input Boolean gyroscopicFeedforward = true
      "Add omega x (I omega) so the moment holds a spinning body on rate";
    output Real moment[3](each unit = "N.m") "Commanded body moment";
  protected
    Real angularMomentum[3](each unit = "kg.m2/s");
  algorithm
    for axis in 1:3 loop
      assert(inertia[axis] > 0.0, "Every body inertia must be positive");
      assert(rateGain[axis] >= 0.0, "Every body-rate gain must be nonnegative");
      moment[axis] := inertia[axis] * rateGain[axis]
        * (angularVelocitySetpoint[axis] - angularVelocity[axis]);
    end for;
    if gyroscopicFeedforward then
      angularMomentum := {
        inertia[1] * angularVelocity[1],
        inertia[2] * angularVelocity[2],
        inertia[3] * angularVelocity[3]};
      moment := moment + cross(angularVelocity, angularMomentum);
    end if;
    annotation(Documentation(info="<html>
      <p>Maps a body-rate error into a body moment for a rigid multirotor:
      <code>M = I (k (omega_sp - omega)) + omega x (I omega)</code>. The
      gyroscopic term is a feed-forward that cancels the rigid-body coupling so
      the proportional term only has to close the tracking error. Inputs and the
      returned moment share the vehicle body frame.</p>
    </html>"));
  end bodyMoment;

  annotation(Documentation(info="<html>
    <p>Inner body-rate loop for multirotors. It converts the angular-velocity
    setpoint produced by an attitude loop into the body moment that the control
    allocator distributes across the rotors.</p>
  </html>"));
end RateLoop;
