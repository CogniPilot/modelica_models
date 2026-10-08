within Vision.Matching;
function RGBDDescriptorValid
  input Real descriptor[:];
  input Real point[3];
  input Boolean enabled;
  output Boolean valid;
protected
  Real energy;
algorithm
  valid := enabled;
  energy := 0.0;
  for k in 1:size(descriptor,1) loop
    valid := valid and abs(descriptor[k]) <= 1.0;
    energy := energy+(if enabled and abs(descriptor[k]) <= 1.0 then descriptor[k]*descriptor[k] else 0.0);
  end for;
  for k in 1:3 loop
    valid := valid and abs(point[k]) <= 1e6;
  end for;
  valid := valid and abs(energy-1.0) <= 1e-8;
end RGBDDescriptorValid;
