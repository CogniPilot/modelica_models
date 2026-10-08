within Vision.Matching;
// Held-IMU intervals return empty descriptors without reading either image.
function DescribeRGBDFrame
  import RGBDCalibratedPoint = Vision.Matching.RGBDCalibratedPoint;
  import RGBDDescriptorConfiguration = Vision.Matching.RGBDDescriptorConfiguration;
  import RGBDDescriptorPosition = Vision.Matching.RGBDDescriptorPosition;
  import RGBDNormalizeDescriptor = Vision.Matching.RGBDNormalizeDescriptor;

  input Real rgb[:,:,:];
  input Real depth[size(rgb,1),size(rgb,2)];
  input Real pixels[:,2];
  input Real activeCount;
  input Real rgbCalibration[4];
  input Real depthCalibration[4];
  input Real disparityNoise;
  input Real noiseReferenceFx;
  input Real baseline;
  input Real nearDepth;
  input Real farDepth;
  input Real minimumContrast;
  input Boolean imageEnabled = true;
  output Real descriptor[size(pixels,1),49];
  output Real point[size(pixels,1),3];
  output Real enabled[size(pixels,1)];
  output Real invalidCount;
  input Real depthUnits = 1.0 "Meters per depth sample; 1 for metric depth, SDK scale for Z16";
protected
  constant Integer colorChannelCount = 3;
  constant Integer descriptorWidth = 7;
  constant Integer descriptorSize = descriptorWidth*descriptorWidth;
  Real patch[descriptorSize];
  Real color[colorChannelCount];
  Real optical[3];
  Real contrastThreshold;
  Integer x;
  Integer y;
  Integer patchIndex;
  Boolean configuration;
  Boolean positionValid;
  Boolean depthValid;
  Boolean valid;
algorithm
  descriptor := zeros(size(pixels,1),descriptorSize);
  point := zeros(size(pixels,1),3);
  enabled := zeros(size(pixels,1));
  invalidCount := 0.0;
  if imageEnabled then
    assert(size(rgb,3) == 3 or size(rgb,3) == 4,"Descriptors expect RGB or RGBA channels");
    configuration := RGBDDescriptorConfiguration({size(rgb,1),size(rgb,2)},{size(depth,1),size(depth,2)},
      activeCount,size(pixels,1),rgbCalibration,nearDepth,farDepth,minimumContrast);
    contrastThreshold := if configuration then descriptorSize*minimumContrast*minimumContrast else 0.0;
    for i in 1:size(pixels,1) loop
      (positionValid,x,y) := RGBDDescriptorPosition(pixels[i,:],{size(rgb,1),size(rgb,2)},configuration and i <= activeCount);
      (optical,depthValid) := RGBDCalibratedPoint(depth,pixels[i,:],positionValid,rgbCalibration,
        depthCalibration,nearDepth,farDepth,disparityNoise,noiseReferenceFx,baseline,depthUnits=depthUnits);
      patch := zeros(descriptorSize);
      // Convert selected patches only; ignore alpha and the rest of the image.
      if positionValid then
        for row in 0:descriptorWidth-1 loop
          for column in 0:descriptorWidth-1 loop
            patchIndex := row*descriptorWidth+column+1;
            color := rgb[y+row-2,x+column-2,1:colorChannelCount];
            patch[patchIndex] := if color[1] >= 0.0 and color[1] <= 255.0
              and color[2] >= 0.0 and color[2] <= 255.0 and color[3] >= 0.0 and color[3] <= 255.0
              then (color[1]+color[2]+color[3])/(colorChannelCount*255.0) else -1.0;
          end for;
        end for;
      end if;
      (descriptor[i,:],valid) := RGBDNormalizeDescriptor(patch,positionValid and depthValid,contrastThreshold);
      point[i,:] := if valid then optical else zeros(3);
      enabled[i] := if valid then 1.0 else 0.0;
      invalidCount := invalidCount+(if i <= activeCount and not valid then 1.0 else 0.0);
    end for;
  end if;
end DescribeRGBDFrame;
