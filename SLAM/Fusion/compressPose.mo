within SLAM.Fusion;

function compressPose "Sufficient pose statistic at one common camera linearization"
  input Real residual[:];
  input Real navigationJacobian[size(residual, 1), 15];
  input Real measurementCovariance[size(residual, 1), size(residual, 1)];
  output Real poseResidual[6];
  output Real poseCovariance[6, 6];
  output Boolean valid;
protected
  Real poseJacobian[size(residual, 1), 6];
  Real weighted[size(residual, 1), 7];
  Real information[6, 6];
  Real solution[6, 7];
  Real noiseRoot[size(residual, 1), size(residual, 1)];
  Boolean noiseValid;
  Boolean weightedValid;
  Boolean informationValid;
algorithm
  poseJacobian := cat(2, navigationJacobian[:, 1:3], navigationJacobian[:, 7:9]);
  (noiseRoot, noiseValid) := LinearAlgebra.factorPSD(measurementCovariance);
  (weighted, weightedValid) := LinearAlgebra.solveSPD(measurementCovariance,
    cat(2, poseJacobian, transpose({residual})));
  information := LinearAlgebra.symmetrize(transpose(poseJacobian) * weighted[:, 1:6]);
  (solution, informationValid) := LinearAlgebra.solveSPD(information,
    cat(2, 1.0 * identity(6), transpose({transpose(poseJacobian) * weighted[:, 7]})));
  poseCovariance := LinearAlgebra.symmetrize(solution[:, 1:6]);
  poseResidual := solution[:, 7];
  valid := noiseValid and weightedValid and informationValid;
end compressPose;
