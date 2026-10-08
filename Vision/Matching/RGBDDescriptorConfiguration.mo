within Vision.Matching;
// Shared patch admission and source-order normalization for gray and RGB inputs.
function RGBDDescriptorConfiguration
  input Integer imageSize[2];
  input Integer depthSize[2];
  input Real activeCount;
  input Integer capacity;
  input Real rgbCalibration[4];
  input Real nearDepth;
  input Real farDepth;
  input Real minimumContrast;
  output Boolean valid;
algorithm
  valid := imageSize[1] == depthSize[1] and imageSize[2] == depthSize[2] and
    activeCount >= 0.0 and activeCount <= capacity and floor(activeCount) == activeCount and
    rgbCalibration[1] >= 1e-6 and rgbCalibration[1] <= 1e6 and rgbCalibration[2] >= 1e-6 and rgbCalibration[2] <= 1e6 and
    abs(rgbCalibration[3]) <= 1e6 and abs(rgbCalibration[4]) <= 1e6 and
    nearDepth > 0.0 and farDepth >= nearDepth and farDepth <= 1e6 and
    minimumContrast > 0.0 and minimumContrast <= 1.0;
end RGBDDescriptorConfiguration;
