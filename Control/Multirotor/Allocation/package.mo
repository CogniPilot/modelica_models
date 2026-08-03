within Control.Multirotor;
package Allocation "Multirotor control allocation"

  function motorCommands
    "Normalized rotor commands from a collective thrust and body moment"
    input Real thrust(unit = "N") "Collective thrust force";
    input Real moment[3](each unit = "N.m") "Commanded body moment";
    input Real wrenchToThrust[4, 4]
      "Maps {thrust, moment} to the four rotor thrusts [N]";
    input Real thrustCoefficient(unit = "N.s2")
      "Rotor thrust coefficient in F = Ct omega^2";
    input Real maxMotorSpeed(unit = "rad/s") "Rotor speed at full command";
    output Real motor[4](each unit = "1")
      "Normalized rotor commands in [0, 1]";
  protected
    Real wrench[4];
    Real motorThrust[4](each unit = "N");
    Real motorSpeed(unit = "rad/s");
  algorithm
    assert(thrustCoefficient > 0.0, "Thrust coefficient must be positive");
    assert(maxMotorSpeed > 0.0, "Maximum rotor speed must be positive");
    wrench := {thrust, moment[1], moment[2], moment[3]};
    motorThrust := wrenchToThrust * wrench;
    for rotor in 1:4 loop
      // A saturated rotor cannot pull, so clamp the demanded thrust to the
      // non-negative producible range before inverting F = Ct omega^2.
      motorSpeed := sqrt(max(0.0, motorThrust[rotor]) / thrustCoefficient);
      motor[rotor] := max(0.0, min(1.0, motorSpeed / maxMotorSpeed));
    end for;
    annotation(Documentation(info="<html>
      <p>Inverts the rotor-thrust-to-wrench map to recover the four rotor
      thrusts from a collective thrust and body moment, then converts each rotor
      thrust to a normalized speed command through the quadratic rotor model.
      <code>wrenchToThrust</code> is the inverse of
      <code>[ones(1,4); momentMap]</code> and is supplied by the vehicle so the
      reusable allocator stays free of any airframe geometry. Negative rotor
      thrusts (a saturated allocation) are clipped to zero.</p>
    </html>"));
  end motorCommands;

  annotation(Documentation(info="<html>
    <p>Control allocation for multirotors. Given the collective thrust and body
    moment from the attitude and rate loops, it produces the per-rotor
    commands the plant consumes.</p>
  </html>"));
end Allocation;
