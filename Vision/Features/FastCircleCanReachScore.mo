within Vision.Features;
// Every FAST-9 arc contains adjacent cardinal samples, including wraparound.
function FastCircleCanReachScore
  import FastCircleStencil = Vision.Features.FastCircleStencil;

  input Real differences[FastCircleStencil.sampleCount];
  input Real scoreFloor;
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
  if scoreFloor > 0.0 and scoreFloor <= 255.0 then
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
    if not possible then
      // Conservatively retain enormous finite values as well as NaN/infinity.
      for slot in 1:size(differences,1) loop
        possible := possible or not (abs(differences[slot]) <= 1e308);
      end for;
    end if;
  end if;
end FastCircleCanReachScore;
