within SLAM.Localization;
// Conditional first-order uncertainty of the UNWEIGHTED point-to-point fit.
// Optical RDF perturbation: delta y = delta t - skew(R*p_ref)*delta theta.
// No correspondence changes, reference-pose uncertainty or cross-pair covariance
// are modeled. See docs/registration-uncertainty.md before filter composition.
function RGBDUncertaintySkew
  input Real v[3];
  output Real S[3,3];
algorithm
  S := {{0.0,-v[3],v[2]},{v[3],0.0,-v[1]},{-v[2],v[1],0.0}};
end RGBDUncertaintySkew;
