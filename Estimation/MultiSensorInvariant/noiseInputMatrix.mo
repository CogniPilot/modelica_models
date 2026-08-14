within Estimation.MultiSensorInvariant;

function noiseInputMatrix "Map continuous IMU and bias noise into the tangent"
  output Real G[TangentLength, ProcessNoiseLength];
algorithm
  G := cat(1,
    zeros(3, ProcessNoiseLength),
    cat(2, zeros(3, 3), -identity(3), zeros(3, 6)),
    cat(2, -identity(3), zeros(3, 9)),
    cat(2, zeros(3, 6), identity(3), zeros(3, 3)),
    cat(2, zeros(3, 9), identity(3)));
end noiseInputMatrix;
