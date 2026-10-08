within Vision.Features;
// Array-level acquisition guard: no grayscale or patch work on held-IMU calls.
function FastFrameScores
  import FastCircleCanReachScore = Vision.Features.FastCircleCanReachScore;
  import FastCircleScore = Vision.Features.FastCircleScore;
  import FastCircleStencil = Vision.Features.FastCircleStencil;

  input Real rgb[:,:,:];
  input Boolean enabled = true;
  input Real scoreFloor = 0.0 "Optional conservative selection floor; zero keeps every score";
  output Real scores[size(rgb,1)*size(rgb,2)];
protected
  constant Integer radius = FastCircleStencil.radius;
  Real gray[size(rgb,1),size(rgb,2)];
  Real differences[FastCircleStencil.sampleCount];
  Real center;
algorithm
  scores := zeros(size(rgb,1)*size(rgb,2));
  if enabled then
    assert(size(rgb,3) == 3 or size(rgb,3) == 4,"FAST expects RGB or RGBA channels");
    for row in 1:size(rgb,1) loop
      for column in 1:size(rgb,2) loop
        gray[row,column] := ((rgb[row,column,1]+rgb[row,column,2])+rgb[row,column,3])/3.0;
      end for;
    end for;
    for row in radius+1:size(rgb,1)-radius loop
      for column in radius+1:size(rgb,2)-radius loop
        center := gray[row,column];
        for sample in 1:FastCircleStencil.sampleCount loop
          differences[sample] := gray[row+FastCircleStencil.offsets[sample,1],
            column+FastCircleStencil.offsets[sample,2]]-center;
        end for;
        if FastCircleCanReachScore(differences,scoreFloor) then
          scores[(row-1)*size(rgb,2)+column] := FastCircleScore(differences);
        end if;
      end for;
    end for;
  end if;
end FastFrameScores;
