within Vision.Features;
// Array-level acquisition guard: no grayscale or patch work on held-IMU calls.
function FastFrameScores
  import FastIntegerFrameScores = Vision.Features.FastIntegerFrameScores;

  input Real rgb[:,:,:] "Finite, integer-valued 8-bit channels in 0..255; integer() floors other values";
  input Boolean enabled = true;
  input Real scoreFloor = 0.0 "Optional conservative selection floor; zero keeps every score";
  output Real scores[size(rgb,1)*size(rgb,2)] "Grey-level scores: exact integer channel-sum scores divided by three";
protected
  Integer sumScores[size(rgb,1)*size(rgb,2)];
  Integer sumFloor;
algorithm
  // Grey = (r+g+b)/3, so a grey-level floor f admits sum differences >= ceil(3 f).
  sumFloor := if scoreFloor > 0.0 and scoreFloor <= 255.0 then integer(ceil(3.0*scoreFloor)) else 0;
  sumScores := FastIntegerFrameScores(rgb,enabled,sumFloor);
  scores := zeros(size(rgb,1)*size(rgb,2));
  if enabled then
    for i in 1:size(scores,1) loop
      scores[i] := sumScores[i]/3.0;
    end for;
  end if;
end FastFrameScores;
