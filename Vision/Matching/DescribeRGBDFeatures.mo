within Vision.Matching;
// Editable image-patch frontend. No pose truth or host matching enters here.
function DescribeRGBDFeatures
  import RGBDCalibratedPoint = Vision.Matching.RGBDCalibratedPoint;
  import RGBDDescriptorConfiguration = Vision.Matching.RGBDDescriptorConfiguration;
  import RGBDDescriptorPosition = Vision.Matching.RGBDDescriptorPosition;
  import RGBDNormalizeDescriptor = Vision.Matching.RGBDNormalizeDescriptor;

  input Real gray[:,:] "Measured grayscale intensity in [0,1]";
  input Real depth[:,:] "Measured axial samples; depthUnits converts to meters";
  input Real pixels[:,2] "Zero-based, integer image coordinates";
  input Real activeCount;
  input Real rgbCalibration[4] "RGB fx,fy,cx,cy";
  input Real depthCalibration[4] "Depth fx,fy,cx,cy";
  input Real disparityNoise;
  input Real noiseReferenceFx;
  input Real baseline;
  input Real nearDepth;
  input Real farDepth;
  input Real minimumContrast;
  output Real descriptor[size(pixels,1),49];
  output Real point[size(pixels,1),3] "Camera frame: right, down, forward";
  output Real enabled[size(pixels,1)];
  output Real invalidCount;
  input Real depthUnits = 1.0 "Meters per depth sample; 1 for metric depth, SDK scale for Z16";
protected
  constant Integer patchWidth = 7;
  constant Integer patchSize = patchWidth*patchWidth;
  Real optical[3];
  Boolean depthValid;
  Real patch[patchSize];
  Real contrastThreshold;
  Integer x;
  Integer y;
  Integer patchIndex;
  Boolean configuration;
  Boolean positionValid;
  Boolean valid;
algorithm
  optical := zeros(3);
  depthValid := false;
  positionValid := false;
  valid := false;
  x := 3;
  y := 3;
  patchIndex := 1;
  patch := zeros(patchSize);
  descriptor := zeros(size(pixels,1),patchSize);
  point := zeros(size(pixels,1),3);
  enabled := zeros(size(pixels,1));
  invalidCount := 0.0;
  configuration := RGBDDescriptorConfiguration({size(gray,1),size(gray,2)},{size(depth,1),size(depth,2)},
    activeCount,size(pixels,1),rgbCalibration,nearDepth,farDepth,minimumContrast);
  contrastThreshold := if configuration then patchSize*minimumContrast*minimumContrast else 0.0;
  for i in 1:size(pixels,1) loop
    (positionValid,x,y) := RGBDDescriptorPosition(pixels[i,:],{size(gray,1),size(gray,2)},configuration and i <= activeCount);
    (optical,depthValid) := RGBDCalibratedPoint(depth,pixels[i,:],positionValid,
      rgbCalibration,depthCalibration,nearDepth,farDepth,
      disparityNoise,noiseReferenceFx,baseline,depthUnits=depthUnits);
    for row in 0:patchWidth-1 loop
      for column in 0:patchWidth-1 loop
        patchIndex := row*patchWidth+column+1;
        patch[patchIndex] := if positionValid then gray[y+row-2,x+column-2] else 0.0;
      end for;
    end for;
    (descriptor[i,:],valid) := RGBDNormalizeDescriptor(patch,positionValid and depthValid,contrastThreshold);
    point[i,1] := if valid then optical[1] else 0.0;
    point[i,2] := if valid then optical[2] else 0.0;
    point[i,3] := if valid then optical[3] else 0.0;
    enabled[i] := if valid then 1.0 else 0.0;
    invalidCount := invalidCount+(if i <= activeCount and not valid then 1.0 else 0.0);
  end for;
end DescribeRGBDFeatures;
