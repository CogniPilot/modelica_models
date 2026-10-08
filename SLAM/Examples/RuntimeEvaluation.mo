within SLAM.Examples;
// Evaluation-only initial-heading map. This source never supplies observations
// or corrections to the estimator. The caller owns previousSquaredError and
// sampleCount so replay/reset and missing-truth frames stay explicit.
model RuntimeEvaluation
  input Real worldPosition[3] = {0.0,0.0,1.5};
  input Real worldQuaternion[4] = {1.0,0.0,0.0,0.0};
  input Real originPosition[3] = {0.0,0.0,1.5};
  input Real originQuaternion[4] = {1.0,0.0,0.0,0.0};
  input Real estimatePosition[3] = {0.0,0.0,0.0};
  input Real estimateQuaternion[4] = {1.0,0.0,0.0,0.0};
  input Real previousSquaredError = 0.0;
  input Real sampleCount = 1.0;
  output Real referencePosition[3];
  output Real referenceQuaternion[4];
  output Real nextSquaredError;
  output Real currentError;
  output Real ate;
  output Real orientationError;
protected
  Real originYaw; Real c; Real s; Real hc; Real hs;
  Real squaredDistance; Real quaternionDot;
equation
  originYaw = atan2(2.0*(originQuaternion[1]*originQuaternion[4]+originQuaternion[2]*originQuaternion[3]),
    originQuaternion[1]^2+originQuaternion[2]^2-originQuaternion[3]^2-originQuaternion[4]^2);
  c = cos(originYaw); s = sin(originYaw);
  hc = cos(originYaw/2.0); hs = sin(originYaw/2.0);
  referencePosition[1] = c*(worldPosition[1]-originPosition[1])+s*(worldPosition[2]-originPosition[2]);
  referencePosition[2] = -s*(worldPosition[1]-originPosition[1])+c*(worldPosition[2]-originPosition[2]);
  referencePosition[3] = worldPosition[3]-originPosition[3];
  referenceQuaternion = {hc*worldQuaternion[1]+hs*worldQuaternion[4],
    hc*worldQuaternion[2]+hs*worldQuaternion[3],hc*worldQuaternion[3]-hs*worldQuaternion[2],
    hc*worldQuaternion[4]-hs*worldQuaternion[1]};
  squaredDistance = sum((estimatePosition[i]-referencePosition[i])^2 for i in 1:3);
  currentError = sqrt(squaredDistance);
  nextSquaredError = previousSquaredError+squaredDistance;
  ate = sqrt(nextSquaredError/sampleCount);
  quaternionDot = abs(sum(estimateQuaternion[i]*referenceQuaternion[i] for i in 1:4));
  orientationError = 2.0*acos(min(1.0,quaternionDot))*180.0/3.141592653589793;
end RuntimeEvaluation;
