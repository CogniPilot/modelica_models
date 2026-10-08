within SLAM.Inertial;
model SLAMCovariancePSD
  import SLAMCovariancePSDCheck = SLAM.Inertial.SLAMCovariancePSDCheck;

  parameter Integer dimension = 21;
  parameter Real relativeTolerance = 1e-12;
  input Real covariance[dimension,dimension];
  output Real valid;
equation
  valid = SLAMCovariancePSDCheck(covariance,relativeTolerance);
end SLAMCovariancePSD;
