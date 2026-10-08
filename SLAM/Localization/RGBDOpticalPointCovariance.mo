within SLAM.Localization;
function RGBDOpticalPointCovariance
  input Real p[3];
  input Real rgbFx; input Real rgbFy;
  input Real localizationSigma; input Real disparitySigma;
  input Real noiseReferenceFx; input Real baseline;
  input Real depthInflation;
  output Real covariance[3,3];
protected
  Real bearing[3]; Real depthSigma;
algorithm
  bearing := {p[1]/p[3],p[2]/p[3],1.0};
  // No reduction by interpolation neighbor count: correlated disparity bound.
  depthSigma := depthInflation*p[3]*p[3]*disparitySigma/noiseReferenceFx/baseline;
  for a in 1:3 loop
    for b in 1:3 loop
      covariance[a,b] := bearing[a]*bearing[b]*depthSigma*depthSigma;
    end for;
  end for;
  covariance[1,1] := covariance[1,1]+(p[3]*localizationSigma/rgbFx)^2;
  covariance[2,2] := covariance[2,2]+(p[3]*localizationSigma/rgbFy)^2;
end RGBDOpticalPointCovariance;
