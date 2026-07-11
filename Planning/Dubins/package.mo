within Planning;
package Dubins
  "Forward-only planar paths with bounded curvature and constant turn radius"
  annotation(Documentation(info="<html>
    <p>Computes the shortest feasible path among LSL, RSR, LSR, RSL, RLR,
    and LRL. Positions use Cartesian x/y coordinates, headings and turn
    parameters use radians, and the turn radius must be positive.</p>
    <p>Use <code>plan</code> to construct a path and <code>evaluate</code> to
    evaluate its pose at a fraction from zero to one.</p>
  </html>"));
end Dubins;
