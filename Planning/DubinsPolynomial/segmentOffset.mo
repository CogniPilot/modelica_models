within Planning.DubinsPolynomial;
function segmentOffset
  "Septic C3 transverse-offset coefficients for one Dubins segment"
  input Integer segmentIndex(min = 1, max = 3);
  input Integer previousActiveSegment
    "Nearest active segment before this one, or 0 when there is none";
  input Integer nextActiveSegment
    "Nearest active segment after this one, or 0 when there is none";
  input Real nominalKappa[3] "Nominal curvature of each segment";
  input Real segmentLength[3] "Physical length of each segment";
  input Real startCurvature "Curvature requested at the path start";
  input Real goalCurvature "Curvature requested at the path end";
  input Real startCurvatureDerivative;
  input Real goalCurvatureDerivative;
  output Real coefficient[8]
    "Power-basis coefficients in local physical nominal distance";
  output Boolean accepted "False when the polynomial solve fails";
protected
  Real desiredStartCurvature;
  Real desiredEndCurvature;
  Real desiredStartCurvatureDerivative;
  Real desiredEndCurvatureDerivative;
  Real startDerivative[4];
  Real endDerivative[4];
algorithm
  desiredStartCurvature := if previousActiveSegment == 0 then
      startCurvature else 0.5 * (nominalKappa[previousActiveSegment]
        + nominalKappa[segmentIndex]);
  desiredEndCurvature := if nextActiveSegment == 0 then
      goalCurvature else 0.5 * (nominalKappa[segmentIndex]
        + nominalKappa[nextActiveSegment]);
  desiredStartCurvatureDerivative := if previousActiveSegment == 0 then
      startCurvatureDerivative else 0.0;
  desiredEndCurvatureDerivative := if nextActiveSegment == 0 then
      goalCurvatureDerivative else 0.0;
  startDerivative := {
    0.0,
    0.0,
    desiredStartCurvature - nominalKappa[segmentIndex],
    desiredStartCurvatureDerivative};
  endDerivative := {
    0.0,
    0.0,
    desiredEndCurvature - nominalKappa[segmentIndex],
    desiredEndCurvatureDerivative};
  (coefficient, accepted) := Polynomials.hermiteCoefficients(
    startDerivative, endDerivative, segmentLength[segmentIndex]);
  annotation(Documentation(info = "<html>
    <p>Split out of <code>smoothOffsets</code> so that the multi-output
    <code>Polynomials.hermiteCoefficients</code> call is reached from a
    straight-line caller rather than from inside a <code>for</code> loop.
    Rumoca miscompiles loop-carried state in any function loop that also
    contains a multi-output call, so the three fixed Dubins segments are
    unrolled at the call site.</p>
  </html>"));
end segmentOffset;
