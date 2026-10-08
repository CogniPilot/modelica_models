within Vision.Features;
// Standalone patch scoring reads only the FAST circle.
function FastPatchScore
  import FastCircleScore = Vision.Features.FastCircleScore;
  import FastCircleStencil = Vision.Features.FastCircleStencil;

  input Real gray[2*FastCircleStencil.radius+1,2*FastCircleStencil.radius+1];
  output Real score;
protected
  constant Integer center = FastCircleStencil.radius+1;
  Real differences[FastCircleStencil.sampleCount];
algorithm
  for sample in 1:FastCircleStencil.sampleCount loop
    differences[sample] := gray[center+FastCircleStencil.offsets[sample,1],
      center+FastCircleStencil.offsets[sample,2]]-gray[center,center];
  end for;
  score := FastCircleScore(differences);
end FastPatchScore;
