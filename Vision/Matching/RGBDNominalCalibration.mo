within Vision.Matching;
// Pinhole intrinsics from image size and horizontal/vertical field of view.
function RGBDNominalCalibration
  input Integer imageSize[2] "Height, width";
  input Real fieldOfViewDegrees[2] "Horizontal, vertical";
  output Real calibration[4] "fx, fy, cx, cy";
algorithm
  calibration := {imageSize[2]/(2.0*tan(fieldOfViewDegrees[1]*3.141592653589793/360.0)),
    imageSize[1]/(2.0*tan(fieldOfViewDegrees[2]*3.141592653589793/360.0)),
    (imageSize[2]-1)/2.0,(imageSize[1]-1)/2.0};
end RGBDNominalCalibration;
