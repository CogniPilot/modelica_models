within Estimation.FusionHorizon;

function predictPose
  "Advance a packed pose with a bias-corrected buffered delta"
  input Real poseVector[10];
  input Real deltaRow[DeltaLength];
  input Real gyroscopeBiasMove_rad_s[3];
  input Real accelerometerBiasMove_m_s2[3];
  input Real gravityWorldEnu_m_s2[3];
  input Boolean correctBias = true;
  output Real predictedVector[10];
protected
  Estimation.FusionHorizon.Delta delta;
  Estimation.FusionHorizon.Delta corrected;
  Estimation.FusionHorizon.Pose pose;
  Estimation.FusionHorizon.Pose predicted;
algorithm
  delta := Estimation.FusionHorizon.unpackDelta(deltaRow);
  corrected := if correctBias then Estimation.FusionHorizon.rebiasDelta(
    delta, gyroscopeBiasMove_rad_s, accelerometerBiasMove_m_s2) else delta;
  pose := Estimation.FusionHorizon.Pose(
    positionWorldEnu_m=poseVector[1:3],
    velocityWorldEnu_m_s=poseVector[4:6],
    quaternionWorldBody=poseVector[7:10]);
  predicted := Estimation.FusionHorizon.composePose(pose, corrected, gravityWorldEnu_m_s2);
  predictedVector := cat(1, predicted.positionWorldEnu_m,
    predicted.velocityWorldEnu_m_s, predicted.quaternionWorldBody);
  annotation(Inline = false);
end predictPose;
