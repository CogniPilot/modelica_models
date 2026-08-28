within Estimation.FusionHorizon;

record Delta
  "One SE_2(3) preintegral right factor with its bias sensitivities"
  Real deltaPositionBodyFlu_m[3](each unit = "m")
    "Closed-form position increment in the body frame at the span start";
  Real deltaVelocityBodyFlu_m_s[3](each unit = "m/s")
    "Sculling-corrected velocity increment in the body frame at the span start";
  Real deltaQuaternionBodyFlu[4](each unit = "1")
    "Scalar-first relative rotation from the span-start to the span-end body frame";
  Real integrationTime_s(unit = "s") "Span covered by this right factor";
  Real deltaRotationGyroscopeBiasJacobian_s[3, 3](each unit = "s");
  Real deltaVelocityGyroscopeBiasJacobian_m[3, 3](each unit = "m");
  Real deltaVelocityAccelerometerBiasJacobian_s[3, 3](each unit = "s");
  Real deltaPositionGyroscopeBiasJacobian_m_s[3, 3](each unit = "m.s");
  Real deltaPositionAccelerometerBiasJacobian_s2[3, 3](each unit = "s2");
end Delta;
