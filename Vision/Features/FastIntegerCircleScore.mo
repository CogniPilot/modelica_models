within Vision.Features;
// Exact FAST-9 score from sixteen integer circle-minus-center differences.
// The same ordered minima/maxima as FastCircleScore, in integer arithmetic.
function FastIntegerCircleScore
  import FastCircleStencil = Vision.Features.FastCircleStencil;

  input Integer differences[FastCircleStencil.sampleCount];
  output Integer score;
protected
  constant Integer circleSize = FastCircleStencil.sampleCount;
  constant Integer arcLength = 9;
  Integer extended[circleSize+arcLength-1];
  Integer low2[circleSize+arcLength-3];
  Integer high2[circleSize+arcLength-3];
  Integer low4[circleSize+arcLength-5];
  Integer high4[circleSize+arcLength-5];
  Integer low8;
  Integer high8;
  Integer bright;
  Integer dark;
  Integer response;
algorithm
  // Reuse ordered minima/maxima for windows of 2, 4, 8, then 9 samples.
  for i in 1:circleSize loop
    extended[i] := differences[i];
  end for;
  for i in 1:arcLength-1 loop
    extended[i+circleSize] := differences[i];
  end for;
  for i in 1:size(low2,1) loop
    low2[i] := if extended[i] < extended[i+1] then extended[i] else extended[i+1];
    high2[i] := if extended[i] > extended[i+1] then extended[i] else extended[i+1];
  end for;
  for i in 1:size(low4,1) loop
    low4[i] := if low2[i] < low2[i+2] then low2[i] else low2[i+2];
    high4[i] := if high2[i] > high2[i+2] then high2[i] else high2[i+2];
  end for;
  // Final arc stages need no intermediate arrays.
  score := 0;
  for arc in 1:circleSize loop
    low8 := if low4[arc] < low4[arc+4] then low4[arc] else low4[arc+4];
    high8 := if high4[arc] > high4[arc+4] then high4[arc] else high4[arc+4];
    bright := if low8 < extended[arc+8] then low8 else extended[arc+8];
    dark := -(if high8 > extended[arc+8] then high8 else extended[arc+8]);
    response := if bright > dark then bright else dark;
    score := if score > response then score else response;
  end for;
end FastIntegerCircleScore;
