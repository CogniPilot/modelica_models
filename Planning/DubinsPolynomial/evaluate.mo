within Planning.DubinsPolynomial;
function evaluate "Evaluate a piecewise transverse-offset path at a global fraction"
  input Planning.Dubins.Path path;
  input Real offsetCoefficient[3, :]
    "One power-basis polynomial per Dubins segment";
  input Real fraction;
  output Planning.DubinsPolynomial.State state;
protected
  Integer selectedSegment;
  Integer lastPositiveSegment;
  Real pathDistance;
  Real accumulatedDistance;
  Real segmentLength;
  Real localDistance;
algorithm
  assert(path.feasible, "Cannot offset an infeasible Dubins path");
  assert(fraction >= 0.0 and fraction <= 1.0,
    "Dubins-polynomial evaluation fraction must be in [0, 1]");
  pathDistance := fraction * path.length;
  selectedSegment := 0;
  lastPositiveSegment := 0;
  accumulatedDistance := 0.0;
  localDistance := pathDistance;
  for segmentIndex in 1:3 loop
    segmentLength := path.turnRadius
      * path.normalizedSegmentLength[segmentIndex];
    if segmentLength > 1.0e-10 then
      lastPositiveSegment := segmentIndex;
    end if;
    if segmentLength > 1.0e-10 and
        pathDistance <= accumulatedDistance + segmentLength + 1.0e-12 and
        selectedSegment == 0 then
      selectedSegment := segmentIndex;
      localDistance := pathDistance - accumulatedDistance;
    end if;
    accumulatedDistance := accumulatedDistance + segmentLength;
  end for;
  assert(lastPositiveSegment > 0,
    "Dubins-polynomial path must contain a positive-length segment");
  if fraction >= 1.0 then
    selectedSegment := lastPositiveSegment;
    localDistance := path.turnRadius
      * path.normalizedSegmentLength[lastPositiveSegment];
  end if;
  state := Planning.DubinsPolynomial.evaluateSegment(
    path.startPosition,
    path.startHeading,
    path.turnRadius,
    path.pathType,
    path.normalizedSegmentLength,
    offsetCoefficient[selectedSegment, :],
    selectedSegment,
    max(0.0, localDistance));
end evaluate;
