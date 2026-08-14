within Control.Multirotor;
package Allocation "Multirotor control allocation"

  function rotorEffectiveness
    "Map individual upward rotor thrusts into collective thrust and body moment"
    input Real positionBodyFlu_m[:, 3]
      "Rotor centers relative to the center of mass";
    input Real yawMomentPerThrust_m[size(positionBodyFlu_m, 1)]
      "Signed reaction-torque ratio; positive produces positive FLU yaw moment";
    output Real effectiveness[4, size(positionBodyFlu_m, 1)]
      "Maps rotor thrust into {collective thrust, body moment}";
  algorithm
    for rotor in 1:size(positionBodyFlu_m, 1) loop
      effectiveness[:, rotor] := {
        1.0,
        positionBodyFlu_m[rotor, 2],
        -positionBodyFlu_m[rotor, 1],
        yawMomentPerThrust_m[rotor]};
    end for;
  end rotorEffectiveness;

  function quadrotorWrenchToThrust
    "Derive the square quadrotor allocation inverse from physical geometry"
    input Real positionBodyFlu_m[4, 3];
    input Real yawMomentPerThrust_m[4];
    output Real wrenchToRotorThrust[4, 4];
  protected
    Real effectiveness[4, 4];
    Boolean accepted;
  algorithm
    effectiveness := rotorEffectiveness(
      positionBodyFlu_m, yawMomentPerThrust_m);
    (wrenchToRotorThrust, accepted) := LinearAlgebra.solve(
      effectiveness, identity(4));
    assert(accepted,
      "Quadrotor geometry must provide independent thrust, roll, pitch, and yaw authority");
  end quadrotorWrenchToThrust;

  function rotorCommands
    "Normalized rotor commands from a collective thrust and body moment"
    input Integer nRotors(min = 1) "Number of independently commanded rotors";
    input Real thrust(unit = "N") "Collective thrust force";
    input Real moment[3](each unit = "N.m") "Commanded body moment";
    input Real wrenchToRotorThrust[nRotors, 4]
      "Maps {thrust, moment} to per-rotor thrust [N]";
    input Real thrustCoefficient[nRotors](each unit = "N.s2")
      "Per-rotor coefficient in F = Ct omega^2";
    input Real maximumRotorSpeed[nRotors](each unit = "rad/s")
      "Per-rotor speed at full command";
    output Real command[nRotors](each unit = "1")
      "Normalized rotor commands in [0, 1]";
  protected
    Real wrench[4];
    Real rotorThrust[nRotors](each unit = "N");
    Real rotorSpeed[nRotors](each unit = "rad/s");
  algorithm
    wrench := {thrust, moment[1], moment[2], moment[3]};
    rotorThrust := wrenchToRotorThrust * wrench;
    for rotor in 1:nRotors loop
      assert(thrustCoefficient[rotor] > 0.0,
        "Every rotor thrust coefficient must be positive");
      assert(maximumRotorSpeed[rotor] > 0.0,
        "Every maximum rotor speed must be positive");
      // A saturated rotor cannot pull, so clamp the demanded thrust to the
      // non-negative producible range before inverting F = Ct omega^2.
      rotorSpeed[rotor] :=
        sqrt(max(0.0, rotorThrust[rotor]) / thrustCoefficient[rotor]);
      command[rotor] := MathUtilities.clip(
        rotorSpeed[rotor] / maximumRotorSpeed[rotor], 0.0, 1.0);
    end for;
    annotation(Documentation(info="<html>
      <p>Applies a vehicle-supplied right inverse of the rotor effectiveness
      tensor, then converts every demanded rotor thrust through its own
      quadratic rotor model. The reusable allocator therefore supports any
      rotor count and heterogeneous propulsion without owning airframe
      geometry. Negative rotor thrusts are clipped to zero.</p>
    </html>"));
  end rotorCommands;

  annotation(Documentation(info="<html>
    <p>Control allocation for multirotors. Given the collective thrust and body
    moment from the attitude and rate loops, it produces the per-rotor
    commands the plant consumes.</p>
  </html>"));
end Allocation;
