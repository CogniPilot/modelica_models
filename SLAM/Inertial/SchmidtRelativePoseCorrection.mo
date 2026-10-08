within SLAM.Inertial;
model SchmidtRelativePoseCorrection
  import ES15CorrectionSolve = SLAM.Inertial.ES15CorrectionSolve;
  import RGBDProperRotation = SLAM.Localization.RGBDProperRotation;
  import SLAMCovariancePSD = SLAM.Inertial.SLAMCovariancePSD;
  import SLAMRotationLog = SLAM.Inertial.SLAMRotationLog;

  constant Integer spaceDimension = 3;
  constant Integer errorDimension = 15;
  constant Integer poseDimension = 2*spaceDimension;
  constant Integer augmentedDimension = errorDimension+poseDimension;
  parameter Real maximumNis = 22.46;
  parameter Real maximumAngularInnovation = 0.35;
  input Real position[spaceDimension] = zeros(spaceDimension);
  input Real velocity[spaceDimension] = zeros(spaceDimension);
  input Real rotation[spaceDimension,spaceDimension] = identity(spaceDimension);
  input Real accelBias[spaceDimension] = zeros(spaceDimension);
  input Real gyroBias[spaceDimension] = zeros(spaceDimension);
  input Real covariance[errorDimension,errorDimension] = identity(errorDimension);
  input Real referencePosition[spaceDimension] = zeros(spaceDimension);
  input Real referenceRotation[spaceDimension,spaceDimension] = identity(spaceDimension);
  input Real referenceCovariance[poseDimension,poseDimension] = identity(poseDimension);
  input Real crossCovariance[errorDimension,poseDimension] = zeros(errorDimension,poseDimension);
  input Real opticalToBody[spaceDimension,spaceDimension] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[spaceDimension] = {0.18,0.0,-0.04};
  input Real measuredRotation[spaceDimension,spaceDimension] = identity(spaceDimension);
  input Real measuredTranslation[spaceDimension] = zeros(spaceDimension);
  // Covariance of independent translation / left-current-optical rotation errors.
  input Real relativeCovariance[poseDimension,poseDimension] = identity(poseDimension);
  input Real measurementEnabled = 0.0;
  output Real accepted;
  output Real nis;
  output Real innovation[poseDimension];
  output Real measurementJacobian[poseDimension,augmentedDimension];
  output Real innovationCovariance[poseDimension,poseDimension];
  output Real nextPosition[spaceDimension];
  output Real nextVelocity[spaceDimension];
  output Real nextRotation[spaceDimension,spaceDimension];
  output Real nextAccelBias[spaceDimension];
  output Real nextGyroBias[spaceDimension];
  output Real nextCovariance[errorDimension,errorDimension];
  output Real nextCrossCovariance[errorDimension,poseDimension];
  output Real nextReferenceCovariance[poseDimension,poseDimension];
protected
  RGBDProperRotation currentCheck(rotation=rotation);
  RGBDProperRotation referenceCheck(rotation=referenceRotation);
  RGBDProperRotation extrinsicsCheck(rotation=opticalToBody);
  RGBDProperRotation measuredCheck(rotation=measuredRotation);
  Real currentCameraTranspose[spaceDimension,spaceDimension];
  Real displacementBody[spaceDimension];
  Real predictedRotation[spaceDimension,spaceDimension];
  Real predictedTranslation[spaceDimension];
  SLAMRotationLog logarithm(rotation=transpose(predictedRotation)*measuredRotation);
  Real skewDisplacement[spaceDimension,spaceDimension];
  Real skewOrigin[spaceDimension,spaceDimension];
  Real skewInnovation[spaceDimension,spaceDimension];
  Real inverseLeft[spaceDimension,spaceDimension];
  Real inverseRight[spaceDimension,spaceDimension];
  Real jacobianCoefficient;
  Real currentTranslationAngle[spaceDimension,spaceDimension];
  Real referenceTranslationAngle[spaceDimension,spaceDimension];
  Real currentRotationAngle[spaceDimension,spaceDimension];
  Real referenceRotationAngle[spaceDimension,spaceDimension];
  Real noiseMap[poseDimension,poseDimension];
  Real noiseRotation[spaceDimension,spaceDimension];
  Real noise[poseDimension,poseDimension];
  Real prior[augmentedDimension,augmentedDimension];
  // The component's default dimension is exactly21. Rebinding it to an equal
  // constant would introduce an unnecessary parameter-initialization owner.
  SLAMCovariancePSD priorCheck(covariance=prior);
  ES15CorrectionSolve noiseCheck(A=relativeCovariance,B=zeros(poseDimension,errorDimension+1));
  Real cross[augmentedDimension,poseDimension];
  Real rhs[poseDimension,errorDimension+1];
  ES15CorrectionSolve solve(A=innovationCovariance,B=rhs);
  Real gain[augmentedDimension,poseDimension];
  Real correction[errorDimension];
  Real residualMap[augmentedDimension,augmentedDimension];
  Real joseph[augmentedDimension,augmentedDimension];
  Real reset[augmentedDimension,augmentedDimension];
  Real resetCovariance[augmentedDimension,augmentedDimension];
  Real injectedRotation[spaceDimension,spaceDimension];
  Real increment[spaceDimension,spaceDimension];
  Real skewCorrection[spaceDimension,spaceDimension];
  Real correctionAngle;
  Real sineCoefficient;
  Real cosineCoefficient;
  Real resetCoefficient;
  Real skewCorrectionSquared[spaceDimension,spaceDimension];
  Real finiteChecks[augmentedDimension,augmentedDimension];
  Real geometryChecks[spaceDimension];
equation
  currentCameraTranspose = transpose(opticalToBody)*transpose(rotation);
  displacementBody = transpose(rotation)*(referencePosition-position+referenceRotation*cameraOriginBody);
  predictedRotation = currentCameraTranspose*referenceRotation*opticalToBody;
  predictedTranslation = transpose(opticalToBody)*(displacementBody-cameraOriginBody);
  innovation = cat(1,measuredTranslation-predictedTranslation,logarithm.vector);
  skewDisplacement = [0.0,-displacementBody[3],displacementBody[2];displacementBody[3],0.0,-displacementBody[1];-displacementBody[2],displacementBody[1],0.0];
  skewOrigin = [0.0,-cameraOriginBody[3],cameraOriginBody[2];cameraOriginBody[3],0.0,-cameraOriginBody[1];-cameraOriginBody[2],cameraOriginBody[1],0.0];
  skewInnovation = [0.0,-innovation[6],innovation[5];innovation[6],0.0,-innovation[4];-innovation[5],innovation[4],0.0];
  jacobianCoefficient = if noEvent(logarithm.angle < 1e-4) then
      1.0/12.0+logarithm.angle^2/720.0
    else (1.0-0.5*logarithm.angle*cos(0.5*logarithm.angle)/max(sin(0.5*logarithm.angle),1e-12))/max(logarithm.angle^2,1e-12);
  inverseLeft = identity(spaceDimension)-0.5*skewInnovation+jacobianCoefficient*(skewInnovation*skewInnovation);
  inverseRight = identity(spaceDimension)+0.5*skewInnovation+jacobianCoefficient*(skewInnovation*skewInnovation);
  currentTranslationAngle = transpose(opticalToBody)*skewDisplacement;
  referenceTranslationAngle = -currentCameraTranspose*referenceRotation*skewOrigin;
  currentRotationAngle = -inverseLeft*transpose(predictedRotation)*transpose(opticalToBody);
  referenceRotationAngle = inverseLeft*transpose(opticalToBody);
  noiseRotation = inverseRight*transpose(measuredRotation);
  // Explicit blocks keep every coordinate valid even during structural projection.
  // H = -d(innovation)/d(error); columns are p,v,theta,ba,bg,p_ref,theta_ref.
  measurementJacobian = cat(1,
    cat(2,-currentCameraTranspose,zeros(3,3),currentTranslationAngle,
      zeros(3,6),currentCameraTranspose,referenceTranslationAngle),
    cat(2,zeros(3,6),currentRotationAngle,zeros(3,9),referenceRotationAngle));
  noiseMap = cat(1,cat(2,identity(3),zeros(3,3)),
    cat(2,zeros(3,3),noiseRotation));
  prior = cat(1,cat(2,covariance,crossCovariance),cat(2,transpose(crossCovariance),referenceCovariance));
  cross = prior*transpose(measurementJacobian);
  noise = noiseMap*relativeCovariance*transpose(noiseMap);
  innovationCovariance = measurementJacobian*cross+noise;
  for i in 1:poseDimension loop
    for j in 1:errorDimension loop
      rhs[i,j] = cross[j,i];
    end for;
    rhs[i,errorDimension+1] = innovation[i];
  end for;
  // Schmidt gain: reference uncertainty enters S, but reference gain is zero.
  gain = cat(1,transpose(solve.X[:,1:errorDimension]),zeros(poseDimension,poseDimension));
  correction = gain[1:errorDimension,:]*innovation;
  nis = sum(innovation[i]*solve.X[i,errorDimension+1] for i in 1:poseDimension);
  residualMap = identity(augmentedDimension)-gain*measurementJacobian;
  joseph = residualMap*prior*transpose(residualMap)+gain*noise*transpose(gain);
  skewCorrection = [0.0,-correction[9],correction[8];correction[9],0.0,-correction[7];-correction[8],correction[7],0.0];
  correctionAngle = sqrt(sum(correction[i]^2 for i in 7:9));
  sineCoefficient = if noEvent(correctionAngle < 1e-7) then 1.0 else sin(correctionAngle)/max(correctionAngle,1e-12);
  cosineCoefficient = if noEvent(correctionAngle < 1e-7) then 0.5 else (1.0-cos(correctionAngle))/max(correctionAngle^2,1e-12);
  resetCoefficient = if noEvent(correctionAngle < 1e-6) then 1.0/6.0 else (correctionAngle-sin(correctionAngle))/max(correctionAngle^3,1e-18);
  skewCorrectionSquared = skewCorrection*skewCorrection;
  increment = identity(spaceDimension)+sineCoefficient*skewCorrection+cosineCoefficient*skewCorrectionSquared;
  injectedRotation = rotation*increment;
  // Only current right-local attitude is reset; frozen reference tangent is unchanged.
  reset = cat(1,
    cat(2,identity(6),zeros(6,15)),
    cat(2,zeros(3,6),identity(3)-cosineCoefficient*skewCorrection
      +resetCoefficient*skewCorrectionSquared,zeros(3,12)),
    cat(2,zeros(12,9),identity(12)));
  resetCovariance = reset*joseph*transpose(reset);
  for i in 1:augmentedDimension loop
    for j in 1:augmentedDimension loop
      finiteChecks[i,j] = if noEvent(abs(resetCovariance[i,j]) <= 1e12) then 0.0 else 1.0;
    end for;
  end for;
  for i in 1:spaceDimension loop
    geometryChecks[i] = if noEvent(abs(position[i]) <= 1e6 and abs(referencePosition[i]) <= 1e6
      and abs(velocity[i]) <= 1e6 and abs(cameraOriginBody[i]) <= 10.0
      and abs(measuredTranslation[i]) <= 1e6 and abs(correction[i]) <= 1e6
      and abs(correction[i+3]) <= 1e6
      and abs(accelBias[i]+correction[i+9]) <= 2.0
      and abs(gyroBias[i]+correction[i+12]) <= 0.3) then 0.0 else 1.0;
  end for;
  accepted = if noEvent(measurementEnabled >= 1.0 and measurementEnabled <= 1.0
    and maximumNis > 0.0 and maximumNis <= 1e6
    and maximumAngularInnovation > 0.0 and maximumAngularInnovation <= 1.0
    and currentCheck.valid > 0.5 and referenceCheck.valid > 0.5
    and extrinsicsCheck.valid > 0.5 and measuredCheck.valid > 0.5 and logarithm.valid > 0.5
    and priorCheck.valid > 0.5 and noiseCheck.valid > 0.5 and solve.valid > 0.5
    and nis >= 0.0 and nis <= maximumNis and logarithm.angle <= maximumAngularInnovation
    and sum(finiteChecks) < 0.5 and sum(geometryChecks) < 0.5) then 1.0 else 0.0;
  nextPosition = if noEvent(accepted > 0.5) then position+correction[1:3] else position;
  nextVelocity = if noEvent(accepted > 0.5) then velocity+correction[4:6] else velocity;
  nextRotation = if noEvent(accepted > 0.5) then injectedRotation else rotation;
  nextAccelBias = if noEvent(accepted > 0.5) then accelBias+correction[10:12] else accelBias;
  nextGyroBias = if noEvent(accepted > 0.5) then gyroBias+correction[13:15] else gyroBias;
  for i in 1:errorDimension loop
    for j in 1:errorDimension loop
      nextCovariance[i,j] = if noEvent(accepted > 0.5) then
        0.5*(resetCovariance[i,j]+resetCovariance[j,i]) else covariance[i,j];
    end for;
    for j in 1:poseDimension loop
      nextCrossCovariance[i,j] = if noEvent(accepted > 0.5) then
        resetCovariance[i,errorDimension+j] else crossCovariance[i,j];
    end for;
  end for;
  // Schmidt gain/reset leave this block and the retained reference pose unchanged.
  nextReferenceCovariance = referenceCovariance;
end SchmidtRelativePoseCorrection;
