within Control;

package Allocation "Geometry-derived control effectiveness and allocation"

  function rotorWrenchEffectiveness
    "Map rotor thrust magnitudes into a full body-frame force and moment wrench"
    input Real positionBody_m[:, 3]
      "Rotor centers relative to the center of mass";
    input Real thrustDirectionBody[size(positionBody_m, 1), 3]
      "Thrust direction for each rotor; normalized internally";
    input Real reactionTorquePerThrust_m[size(positionBody_m, 1)]
      "Signed reaction torque along each rotor thrust axis";
    output Real effectiveness[6, size(positionBody_m, 1)]
      "Maps rotor thrust into {forceBody, momentBody}";
  protected
    Real directionNorm;
    Real direction[3];
  algorithm
    for rotor in 1:size(positionBody_m, 1) loop
      directionNorm := sqrt(sum(
        thrustDirectionBody[rotor, axis] ^ 2 for axis in 1:3));
      assert(directionNorm > 1.0e-12,
        "Every rotor must have a nonzero thrust direction");
      direction := thrustDirectionBody[rotor, :] / directionNorm;
      effectiveness[1:3, rotor] := direction;
      effectiveness[4:6, rotor] :=
        cross(positionBody_m[rotor, :], direction)
          + reactionTorquePerThrust_m[rotor] * direction;
    end for;
  end rotorWrenchEffectiveness;

  function minimumNormRightInverse
    "Right inverse E'*(E*E')^-1 for a full-row-rank effectiveness matrix"
    input Real effectiveness[:, :];
    output Real allocation[size(effectiveness, 2), size(effectiveness, 1)];
  protected
    Integer wrenchDimension = size(effectiveness, 1);
    Integer actuatorCount = size(effectiveness, 2);
    Real gram[wrenchDimension, wrenchDimension];
    Real gramInverse[wrenchDimension, wrenchDimension];
    Boolean accepted;
  algorithm
    assert(actuatorCount >= wrenchDimension,
      "A right inverse requires at least as many actuators as controlled wrench axes");
    gram := effectiveness * transpose(effectiveness);
    (gramInverse, accepted) := LinearAlgebra.solve(
      gram, identity(wrenchDimension));
    assert(accepted,
      "Actuator geometry must provide independent authority in every controlled wrench axis");
    allocation := transpose(effectiveness) * gramInverse;
  end minimumNormRightInverse;

  annotation(Documentation(info = "<html>
    <p>This package owns vehicle-independent actuator geometry. Fixed-axis
    multirotors select body-Z force plus three moment rows. VTOL aircraft may
    retain all six wrench rows and update thrust directions as nacelles tilt.
    Vehicle packages supply only positions, directions, reaction-torque signs,
    limits, and task timing.</p>
  </html>"));
end Allocation;
