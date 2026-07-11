within Planning;
package DubinsPolynomial
  "Smooth transverse-polynomial offsets from nominal Dubins paths"
  annotation(Documentation(info="<html>
    <p>Represents a trajectory as a lateral polynomial offset from each of the
    three segments of a nominal Dubins path. Polynomial abscissas are physical
    nominal path distance, so derivatives have consistent metric units.</p>
    <p>The evaluator returns true unit-speed spatial derivatives through third
    order, signed curvature, and curvature derivative. Arbitrary coefficient
    counts are supported. <code>smoothOffsets</code> provides a septic C3 seed
    with zero offset and heading correction at segment junctions.</p>
  </html>"));
end DubinsPolynomial;
