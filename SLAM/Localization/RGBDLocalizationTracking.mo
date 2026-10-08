within SLAM.Localization;
function RGBDLocalizationTracking
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;

  input Real index[:]; input Real currentPixels[:,2]; input Real oldPixels[size(index,1),2];
  input Real oldEnabled[size(index,1)]; input Real currentEnabled[size(currentPixels,1)];
  input Integer imageSize[2] "Shared reference/current RGB grid: height, width";
  output Real currentPixel[size(index,1),2]; output Real referencePixel[size(index,1),2];
  output Real enabled[size(index,1)];
protected
  Integer partner;
  Boolean valid;
algorithm
  currentPixel := zeros(size(index,1),2); referencePixel := zeros(size(index,1),2);
  enabled := zeros(size(index,1)); partner := 1; valid := false;
  for i in 1:size(index,1) loop
    valid := imageSize[1] > 0 and imageSize[2] > 0
      and index[i] >= 1.0 and index[i] <= size(currentPixels,1)
      and SLAMExactRealEqual(index[i],floor(index[i])) and SLAMExactRealEqual(oldEnabled[i],1.0);
    partner := if valid then integer(index[i]) else 1;
    if valid then
      valid := SLAMExactRealEqual(currentEnabled[partner],1.0);
      for coordinate in 1:2 loop
        valid := valid and oldPixels[i,coordinate] >= 0.0
          and oldPixels[i,coordinate] <= imageSize[3-coordinate]-1
          and SLAMExactRealEqual(oldPixels[i,coordinate],floor(oldPixels[i,coordinate]))
          and currentPixels[partner,coordinate] >= 0.0
          and currentPixels[partner,coordinate] <= imageSize[3-coordinate]-1
          and SLAMExactRealEqual(currentPixels[partner,coordinate],floor(currentPixels[partner,coordinate]));
      end for;
      if valid then
        currentPixel[i,:] := currentPixels[partner,:];
        referencePixel[i,:] := oldPixels[i,:]; enabled[i] := 1.0;
      end if;
    end if;
  end for;
end RGBDLocalizationTracking;
