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
    "Axis-prioritized, desaturating normalized rotor commands"
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
    Real maximumRotorThrust[nRotors](each unit = "N");
    Real collectiveRotorThrust[nRotors](each unit = "N");
    Real rollPitchRotorDelta[nRotors](each unit = "N");
    Real yawRotorDelta[nRotors](each unit = "N");
    Real rotorThrust[nRotors](each unit = "N");
    Real rotorSpeed[nRotors](each unit = "rad/s");
    Real collectiveScale;
    Real rollPitchScale;
    Real yawScale;
  algorithm
    for rotor in 1:nRotors loop
      assert(thrustCoefficient[rotor] > 0.0,
        "Every rotor thrust coefficient must be positive");
      assert(maximumRotorSpeed[rotor] > 0.0,
        "Every maximum rotor speed must be positive");
      assert(wrenchToRotorThrust[rotor, 1] >= 0.0,
        "Collective thrust must not demand negative rotor thrust");
      maximumRotorThrust[rotor] := thrustCoefficient[rotor]
        * maximumRotorSpeed[rotor] * maximumRotorSpeed[rotor];
    end for;

    // Collective has first priority. Scale it only when the requested thrust
    // alone exceeds a rotor's physical range.
    collectiveRotorThrust := wrenchToRotorThrust[:, 1] * max(thrust, 0.0);
    collectiveScale := 1.0;
    for rotor in 1:nRotors loop
      if collectiveRotorThrust[rotor] > maximumRotorThrust[rotor] then
        collectiveScale := min(collectiveScale,
          maximumRotorThrust[rotor] / collectiveRotorThrust[rotor]);
      end if;
    end for;
    collectiveRotorThrust := collectiveScale * collectiveRotorThrust;

    // Roll and pitch keep the second priority because they maintain the
    // thrust-vector attitude. Apply both with one scale to preserve direction.
    rollPitchRotorDelta := wrenchToRotorThrust[:, 2] * moment[1]
      + wrenchToRotorThrust[:, 3] * moment[2];
    rollPitchScale := 1.0;
    for rotor in 1:nRotors loop
      if rollPitchRotorDelta[rotor] > 0.0 then
        rollPitchScale := min(rollPitchScale,
          max(maximumRotorThrust[rotor]
            - collectiveRotorThrust[rotor], 0.0)
              / rollPitchRotorDelta[rotor]);
      elseif rollPitchRotorDelta[rotor] < 0.0 then
        rollPitchScale := min(rollPitchScale,
          max(collectiveRotorThrust[rotor], 0.0)
            / (-rollPitchRotorDelta[rotor]));
      end if;
    end for;
    rotorThrust := collectiveRotorThrust
      + rollPitchScale * rollPitchRotorDelta;

    // Yaw is least important for immediate vehicle stability. Use only the
    // headroom left after collective, roll, and pitch have been allocated.
    yawRotorDelta := wrenchToRotorThrust[:, 4] * moment[3];
    yawScale := 1.0;
    for rotor in 1:nRotors loop
      if yawRotorDelta[rotor] > 0.0 then
        yawScale := min(yawScale,
          max(maximumRotorThrust[rotor] - rotorThrust[rotor], 0.0)
            / yawRotorDelta[rotor]);
      elseif yawRotorDelta[rotor] < 0.0 then
        yawScale := min(yawScale,
          max(rotorThrust[rotor], 0.0) / (-yawRotorDelta[rotor]));
      end if;
    end for;
    rotorThrust := rotorThrust + yawScale * yawRotorDelta;

    for rotor in 1:nRotors loop
      rotorThrust[rotor] := MathUtilities.clip(rotorThrust[rotor], 0.0,
        maximumRotorThrust[rotor]);
      rotorSpeed[rotor] := sqrt(
        rotorThrust[rotor] / thrustCoefficient[rotor]);
      command[rotor] := rotorSpeed[rotor] / maximumRotorSpeed[rotor];
    end for;
    annotation(Documentation(info="<html>
      <p>Applies a vehicle-supplied right inverse while preserving collective
      thrust first, roll/pitch moment second, and yaw moment last. Each group
      is uniformly desaturated against the physical thrust range before the
      quadratic rotor model is inverted. An infeasible yaw request therefore
      cannot corrupt the roll and pitch moments that maintain attitude.</p>
    </html>"));
  end rotorCommands;

  annotation(Documentation(info="<html>
    <p>Control allocation for multirotors. Given the collective thrust and body
    moment from the attitude and rate loops, it produces the per-rotor
    commands the plant consumes.</p>
  </html>"));
end Allocation;
