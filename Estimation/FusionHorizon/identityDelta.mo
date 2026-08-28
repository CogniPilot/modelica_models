within Estimation.FusionHorizon;

function identityDelta "The group identity over a zero span"
  output Estimation.FusionHorizon.Delta delta;
algorithm
  delta := Estimation.FusionHorizon.Delta(
    deltaPositionBodyFlu_m=zeros(3),
    deltaVelocityBodyFlu_m_s=zeros(3),
    deltaQuaternionBodyFlu={1.0, 0.0, 0.0, 0.0},
    integrationTime_s=0.0,
    deltaRotationGyroscopeBiasJacobian_s=zeros(3, 3),
    deltaVelocityGyroscopeBiasJacobian_m=zeros(3, 3),
    deltaVelocityAccelerometerBiasJacobian_s=zeros(3, 3),
    deltaPositionGyroscopeBiasJacobian_m_s=zeros(3, 3),
    deltaPositionAccelerometerBiasJacobian_s2=zeros(3, 3));
end identityDelta;
