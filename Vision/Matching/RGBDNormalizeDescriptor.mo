within Vision.Matching;
function RGBDNormalizeDescriptor
  input Real samples[patchSize];
  input Boolean eligible;
  input Real contrastThreshold;
  output Real descriptor[patchSize];
  output Boolean valid;
protected
  constant Integer patchWidth = 7;
  constant Integer patchSize = patchWidth*patchWidth;
  Real patch[patchSize];
  Real mean;
  Real energy;
  Real scale;
algorithm
  valid := eligible;
  mean := 0.0;
  energy := 0.0;
  for k in 1:patchSize loop
    patch[k] := if samples[k] >= 0.0 and samples[k] <= 1.0 then samples[k] else 0.0;
    valid := valid and samples[k] >= 0.0 and samples[k] <= 1.0;
    mean := mean+patch[k];
  end for;
  mean := mean/patchSize;
  for k in 1:patchSize loop
    patch[k] := patch[k]-mean;
    energy := energy+patch[k]*patch[k];
  end for;
  valid := valid and energy >= contrastThreshold;
  scale := if valid then sqrt(energy) else 1.0;
  for k in 1:patchSize loop
    descriptor[k] := if valid then patch[k]/scale else 0.0;
  end for;
end RGBDNormalizeDescriptor;
