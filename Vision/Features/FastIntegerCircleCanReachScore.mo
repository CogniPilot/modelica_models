within Vision.Features;
// Integer form of FastCircleCanReachScore: every FAST-9 arc contains adjacent
// cardinal samples, including wraparound. A nonpositive floor keeps every score.
function FastIntegerCircleCanReachScore
  import FastCircleStencil = Vision.Features.FastCircleStencil;

  input Integer differences[FastCircleStencil.sampleCount];
  input Integer scoreFloor "Floor in the same integer units as the differences";
  output Boolean possible;
protected
  constant Integer cardinalCount = 4;
  constant Integer stride = div(FastCircleStencil.sampleCount,cardinalCount);
  Integer sample;
  Boolean firstBright;
  Boolean firstDark;
  Boolean lastBright;
  Boolean lastDark;
  Boolean bright;
  Boolean dark;
algorithm
  possible := true;
  if scoreFloor > 0 then
    firstBright := differences[1] >= scoreFloor;
    firstDark := -differences[1] >= scoreFloor;
    lastBright := firstBright;
    lastDark := firstDark;
    possible := false;
    for cardinal in 2:cardinalCount loop
      sample := 1+(cardinal-1)*stride;
      bright := differences[sample] >= scoreFloor;
      dark := -differences[sample] >= scoreFloor;
      possible := possible or (lastBright and bright) or (lastDark and dark);
      lastBright := bright;
      lastDark := dark;
    end for;
    possible := possible or (lastBright and firstBright) or (lastDark and firstDark);
  end if;
end FastIntegerCircleCanReachScore;
