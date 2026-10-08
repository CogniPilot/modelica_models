within Vision.Matching;
model RGBDDescriptorFrame
  import DescribeRGBDFrame = Vision.Matching.DescribeRGBDFrame;

  parameter Integer imageHeight(min=1) = 90;
  parameter Integer imageWidth(min=1) = 160;
  constant Integer featureCapacity = 350;
  constant Integer descriptorWidth = 7;
  constant Integer descriptorSize = descriptorWidth*descriptorWidth;
  parameter Real minimumContrast = 1e-6;
  parameter Real nearDepth = 0.28;
  parameter Real farDepth = 10.0;
  parameter Integer channelCount(min=3,max=4) = 4;
  constant Integer colorChannelCount = 3;
  input Boolean imageEnabled = true "Exactly the camera acquisition, not a held-IMU interval";
  input Real rgb[imageHeight,imageWidth,channelCount];
  input Real depth[imageHeight,imageWidth];
  input Real depthUnits = 1.0 "Meters per depth sample";
  input Real pixels[featureCapacity,2];
  input Real activeCount;
  input Real rgbCalibration[4] = {imageWidth/(2*tan(69*3.141592653589793/360)),imageHeight/(2*tan(42*3.141592653589793/360)),(imageWidth-1)/2.0,(imageHeight-1)/2.0};
  input Real depthCalibration[4] = {imageWidth/(2*tan(87*3.141592653589793/360)),imageHeight/(2*tan(58*3.141592653589793/360)),(imageWidth-1)/2.0,(imageHeight-1)/2.0};
  constant Integer noiseReferenceWidth = 848;
  input Real disparityNoise = 0.1;
  input Real noiseReferenceFx = noiseReferenceWidth/(2*tan(87*3.141592653589793/360));
  input Real baseline = 0.05;
  output Real descriptor[featureCapacity,descriptorSize];
  output Real point[featureCapacity,3];
  output Real enabled[featureCapacity];
  output Real invalidCount;
equation
  (descriptor,point,enabled,invalidCount) = DescribeRGBDFrame(rgb,depth,pixels,activeCount,
    rgbCalibration,depthCalibration,disparityNoise,noiseReferenceFx,baseline,
    nearDepth,farDepth,minimumContrast,imageEnabled,depthUnits=depthUnits);
end RGBDDescriptorFrame;
