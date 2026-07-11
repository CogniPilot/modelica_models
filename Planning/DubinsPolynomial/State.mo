within Planning.DubinsPolynomial;
record State "Geometry and metric derivatives of a transverse-offset path"
  Real position[2];
  Real heading;
  Real firstDerivative[2] "Unit tangent d(position)/ds";
  Real secondDerivative[2] "d2(position)/ds2";
  Real thirdDerivative[2] "d3(position)/ds3";
  Real curvature "Signed curvature";
  Real curvatureDerivative "Derivative of signed curvature with metric distance";
  Real offset "Left-normal offset from the nominal path";
  Real offsetDerivative[4]
    "P, P', P'', and P''' with respect to nominal metric distance";
  Real metricScale "ds_offset/ds_nominal";
  Integer segmentIndex;
end State;
