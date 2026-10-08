within SLAM.Fusion;

function correctMappedLandmarksTight "Fuse raw independent fixed-map RGB-D observations"
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
  Boolean geometryValid;
  Integer rejectionReason;
algorithm
  (residual, navigationJacobian, geometryValid) := linearizeMappedLandmarks(
    predicted, landmarksWorld_m, observations, rotationBodyCamera, cameraPositionBody_m, intrinsics);
  corrected := Estimation.StrapdownINS.ESKF.copyState(predicted);
  accepted := false;
  normalizedInnovationSquared := 0.0;
  if geometryValid then
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      Estimation.StrapdownINS.ESKF.correctLinear(predicted, residual,
        navigationJacobian, measurementCovariance);
  end if;
end correctMappedLandmarksTight;
