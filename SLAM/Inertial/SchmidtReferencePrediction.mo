within SLAM.Inertial;
// Correlated reference lifecycle, composed with the existing ES15/Schmidt models.
// All numerical work is Modelica; the host only retains and copies returned state.
// Current errors: world dp,dv; right-local dtheta; body dba,dbg. Reference: dp,dtheta.
// This source is not yet compiler-admitted or integrated into the production node.
model SchmidtReferencePrediction
  import SchmidtPredictCovariance = SLAM.Inertial.SchmidtPredictCovariance;

  constant Integer currentDimension = 15;
  constant Integer referenceDimension = 6;
  input Real covariance[currentDimension,currentDimension];
  input Real crossCovariance[currentDimension,referenceDimension];
  input Real referenceCovariance[referenceDimension,referenceDimension];
  input Real transition[currentDimension,currentDimension] = identity(currentDimension);
  input Real processCovariance[currentDimension,currentDimension] = zeros(currentDimension,currentDimension);
  input Real referenceAvailable = 0.0;
  input Real predictionEnabled = 1.0;
  output Real accepted;
  output Real nextCovariance[currentDimension,currentDimension];
  output Real nextCrossCovariance[currentDimension,referenceDimension];
  output Real nextReferenceCovariance[referenceDimension,referenceDimension];
algorithm
  (accepted,nextCovariance,nextCrossCovariance,nextReferenceCovariance)
    := SchmidtPredictCovariance(covariance,crossCovariance,referenceCovariance,
      transition,processCovariance,referenceAvailable,predictionEnabled);
end SchmidtReferencePrediction;
