within Vision.Features;
// Ordered FAST-9 score from the sixteen circle-minus-center differences.
// Strict comparisons retain the second operand on ties, including signed zero.
function FastCircleScore
  import FastCircleStencil = Vision.Features.FastCircleStencil;

  input Real differences[FastCircleStencil.sampleCount];
  output Real score;
protected
  constant Integer circleSize = FastCircleStencil.sampleCount;
  constant Integer arcLength = 9;
  Real extended[circleSize+arcLength-1];
  Real low2[circleSize+arcLength-3];
  Real high2[circleSize+arcLength-3];
  Real low4[circleSize+arcLength-5];
  Real high4[circleSize+arcLength-5];
  Real low8;
  Real high8;
  Real bright;
  Real dark;
  Real response;
algorithm
  // Reuse ordered minima/maxima for windows of 2, 4, 8, then 9 samples.
  for i in 1:circleSize loop
    extended[i] := differences[i];
  end for;
  for i in 1:arcLength-1 loop
    extended[i+circleSize] := differences[i];
  end for;
  for i in 1:size(low2,1) loop
    low2[i] := if noEvent(extended[i] < extended[i+1]) then extended[i] else extended[i+1];
    high2[i] := if noEvent(extended[i] > extended[i+1]) then extended[i] else extended[i+1];
  end for;
  for i in 1:size(low4,1) loop
    low4[i] := if noEvent(low2[i] < low2[i+2]) then low2[i] else low2[i+2];
    high4[i] := if noEvent(high2[i] > high2[i+2]) then high2[i] else high2[i+2];
  end for;
  // Final arc stages need no intermediate arrays.
  score := 0.0;
  for arc in 1:circleSize loop
    low8 := if noEvent(low4[arc] < low4[arc+4]) then low4[arc] else low4[arc+4];
    high8 := if noEvent(high4[arc] > high4[arc+4]) then high4[arc] else high4[arc+4];
    bright := if noEvent(low8 < extended[arc+8]) then low8 else extended[arc+8];
    dark := -(if noEvent(high8 > extended[arc+8]) then high8 else extended[arc+8]);
    response := if noEvent(bright > dark) then bright else dark;
    score := if noEvent(score > response) then score else response;
  end for;
end FastCircleScore;
