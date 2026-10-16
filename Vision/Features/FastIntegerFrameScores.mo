within Vision.Features;
// Exact integer FAST-9 scores on channel sums s = r+g+b of an 8-bit raster.
// Scores and floor are in sum units (three times grey levels); every target
// (scalar, SIMD, GPU) computes identical values.
function FastIntegerFrameScores
  import FastIntegerCircleCanReachScore = Vision.Features.FastIntegerCircleCanReachScore;
  import FastIntegerCircleScore = Vision.Features.FastIntegerCircleScore;
  import FastCircleStencil = Vision.Features.FastCircleStencil;

  input Real rgb[:,:,:] "Finite, integer-valued 8-bit channels in 0..255; integer() floors other values";
  input Boolean enabled = true;
  input Integer scoreFloor = 0 "Selection floor in sum units; zero keeps every score";
  output Integer scores[size(rgb,1)*size(rgb,2)];
protected
  constant Integer radius = FastCircleStencil.radius;
  Integer sums[size(rgb,1),size(rgb,2)];
  Integer differences[FastCircleStencil.sampleCount];
  Integer center;
algorithm
  scores := fill(0,size(rgb,1)*size(rgb,2));
  if enabled then
    assert(size(rgb,3) == 3 or size(rgb,3) == 4,"FAST expects RGB or RGBA channels");
    for row in 1:size(rgb,1) loop
      for column in 1:size(rgb,2) loop
        sums[row,column] := integer(rgb[row,column,1])+integer(rgb[row,column,2])+integer(rgb[row,column,3]);
      end for;
    end for;
    for row in radius+1:size(rgb,1)-radius loop
      for column in radius+1:size(rgb,2)-radius loop
        center := sums[row,column];
        for sample in 1:FastCircleStencil.sampleCount loop
          differences[sample] := sums[row+FastCircleStencil.offsets[sample,1],
            column+FastCircleStencil.offsets[sample,2]]-center;
        end for;
        if FastIntegerCircleCanReachScore(differences,scoreFloor) then
          scores[(row-1)*size(rgb,2)+column] := FastIntegerCircleScore(differences);
        end if;
      end for;
    end for;
  end if;
end FastIntegerFrameScores;
