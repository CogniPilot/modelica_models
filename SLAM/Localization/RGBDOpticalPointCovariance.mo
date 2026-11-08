within SLAM.Localization;
function RGBDOpticalPointCovariance
  input Real p[3];
  input Real rgbFx; input Real rgbFy;
  input Real localizationSigma; input Real disparitySigma;
  input Real noiseReferenceFx; input Real baseline;
  input Real depthInflation;
  output Real covariance[3,3];
protected
  Real bearing[3]; Real depthSigma; Real packedCovariance[6];
algorithm
  packedCovariance := zeros(6);
  bearing := {p[1]/p[3],p[2]/p[3],1.0};
  // No reduction by interpolation neighbor count: correlated disparity bound.
  depthSigma := depthInflation*p[3]*p[3]*disparitySigma/noiseReferenceFx/baseline;
  for a in 1:3 loop
    for b in a:3 loop
      packedCovariance[LinearAlgebra.SymmetricMatrix.index(a,b,3)] :=
        bearing[a]*bearing[b]*depthSigma*depthSigma;
    end for;
  end for;
  packedCovariance[1] := packedCovariance[1]+(p[3]*localizationSigma/rgbFx)^2;
  packedCovariance[4] := packedCovariance[4]+(p[3]*localizationSigma/rgbFy)^2;
  covariance := LinearAlgebra.SymmetricMatrix.unpack(3,packedCovariance);
end RGBDOpticalPointCovariance;
