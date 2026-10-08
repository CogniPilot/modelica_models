within Vision.Features;
// Keep two rank bins below the threshold to preserve selector admission.
function FastSelectionScoreFloor
  input Real absoluteThreshold;
  input Real rankScale;
  output Real scoreFloor;
algorithm
  scoreFloor := 0.0;
  if absoluteThreshold > 0.0 and absoluteThreshold <= 255.0
      and rankScale >= 1.0 and rankScale <= 1e12 then
    scoreFloor := max(0.0,(floor(absoluteThreshold*rankScale)-2.0)/rankScale);
  end if;
end FastSelectionScoreFloor;
