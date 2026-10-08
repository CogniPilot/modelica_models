within SLAM.Fusion;

function correctMappedLandmarksLoose "Fuse a compressed pose from independent fixed-map RGB-D observations"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Real landmarksWorld_m[:, 3];
  input Real observations[size(landmarksWorld_m, 1), 3];
  input Real measurementCovariance[3 * size(landmarksWorld_m, 1), 3 * size(landmarksWorld_m, 1)];
  input Real rotationBodyCamera[3, 3];
  input Real cameraPositionBody_m[3];
  input Real intrinsics[4];
  output Estimation.StrapdownINS.ESKF.State corrected;
  output Boolean accepted;
  output Real normalizedInnovationSquared;
protected
  Real residual[3 * size(landmarksWorld_m, 1)];
  Real navigationJacobian[3 * size(landmarksWorld_m, 1), 15];
  Real poseResidual[6];
  Real poseCovariance[6, 6];
  Real poseSelector[6, 15];
  Boolean geometryValid;
  Boolean compressionValid;
  Integer rejectionReason;
algorithm
  (residual, navigationJacobian, geometryValid) := linearizeMappedLandmarks(
    predicted, landmarksWorld_m, observations, rotationBodyCamera, cameraPositionBody_m, intrinsics);
  (poseResidual, poseCovariance, compressionValid) := compressPose(
    residual, navigationJacobian, measurementCovariance);
  poseSelector := zeros(6, 15);
  poseSelector[1:3, 1:3] := 1.0 * identity(3);
  poseSelector[4:6, 7:9] := 1.0 * identity(3);
  corrected := Estimation.StrapdownINS.ESKF.copyState(predicted);
  accepted := false;
  normalizedInnovationSquared := 0.0;
  if geometryValid and compressionValid then
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      Estimation.StrapdownINS.ESKF.correctLinear(predicted, poseResidual,
        poseSelector, poseCovariance);
  end if;
end correctMappedLandmarksLoose;
