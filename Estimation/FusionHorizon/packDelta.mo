within Estimation.FusionHorizon;

function packDelta "Flatten one delta into a ring row"
  input Estimation.FusionHorizon.Delta delta;
  output Real row[DeltaLength];
algorithm
  // The ring is one dense Real matrix rather than nine parallel arrays so the
  // whole buffer is a single contiguous object in generated code and a store
  // is one row. The layout is fixed here and read back by unpackDelta; the two
  // are the only places that know it.
  //
  // Written out element by element rather than as a nested loop over the 3x3
  // blocks. That is not style: measured, the loop form made the code generator
  // emit a three-element copy call per element and cost about twenty thousand
  // instructions per unpack, which dominated every buffer walk. Flat
  // assignments cost the assignments.
  row[1] := delta.deltaPositionBodyFlu_m[1];
  row[2] := delta.deltaPositionBodyFlu_m[2];
  row[3] := delta.deltaPositionBodyFlu_m[3];
  row[4] := delta.deltaVelocityBodyFlu_m_s[1];
  row[5] := delta.deltaVelocityBodyFlu_m_s[2];
  row[6] := delta.deltaVelocityBodyFlu_m_s[3];
  row[7] := delta.deltaQuaternionBodyFlu[1];
  row[8] := delta.deltaQuaternionBodyFlu[2];
  row[9] := delta.deltaQuaternionBodyFlu[3];
  row[10] := delta.deltaQuaternionBodyFlu[4];
  row[11] := delta.integrationTime_s;
  row[12] := delta.deltaRotationGyroscopeBiasJacobian_s[1, 1];
  row[13] := delta.deltaRotationGyroscopeBiasJacobian_s[1, 2];
  row[14] := delta.deltaRotationGyroscopeBiasJacobian_s[1, 3];
  row[15] := delta.deltaRotationGyroscopeBiasJacobian_s[2, 1];
  row[16] := delta.deltaRotationGyroscopeBiasJacobian_s[2, 2];
  row[17] := delta.deltaRotationGyroscopeBiasJacobian_s[2, 3];
  row[18] := delta.deltaRotationGyroscopeBiasJacobian_s[3, 1];
  row[19] := delta.deltaRotationGyroscopeBiasJacobian_s[3, 2];
  row[20] := delta.deltaRotationGyroscopeBiasJacobian_s[3, 3];
  row[21] := delta.deltaVelocityGyroscopeBiasJacobian_m[1, 1];
  row[22] := delta.deltaVelocityGyroscopeBiasJacobian_m[1, 2];
  row[23] := delta.deltaVelocityGyroscopeBiasJacobian_m[1, 3];
  row[24] := delta.deltaVelocityGyroscopeBiasJacobian_m[2, 1];
  row[25] := delta.deltaVelocityGyroscopeBiasJacobian_m[2, 2];
  row[26] := delta.deltaVelocityGyroscopeBiasJacobian_m[2, 3];
  row[27] := delta.deltaVelocityGyroscopeBiasJacobian_m[3, 1];
  row[28] := delta.deltaVelocityGyroscopeBiasJacobian_m[3, 2];
  row[29] := delta.deltaVelocityGyroscopeBiasJacobian_m[3, 3];
  row[30] := delta.deltaVelocityAccelerometerBiasJacobian_s[1, 1];
  row[31] := delta.deltaVelocityAccelerometerBiasJacobian_s[1, 2];
  row[32] := delta.deltaVelocityAccelerometerBiasJacobian_s[1, 3];
  row[33] := delta.deltaVelocityAccelerometerBiasJacobian_s[2, 1];
  row[34] := delta.deltaVelocityAccelerometerBiasJacobian_s[2, 2];
  row[35] := delta.deltaVelocityAccelerometerBiasJacobian_s[2, 3];
  row[36] := delta.deltaVelocityAccelerometerBiasJacobian_s[3, 1];
  row[37] := delta.deltaVelocityAccelerometerBiasJacobian_s[3, 2];
  row[38] := delta.deltaVelocityAccelerometerBiasJacobian_s[3, 3];
  row[39] := delta.deltaPositionGyroscopeBiasJacobian_m_s[1, 1];
  row[40] := delta.deltaPositionGyroscopeBiasJacobian_m_s[1, 2];
  row[41] := delta.deltaPositionGyroscopeBiasJacobian_m_s[1, 3];
  row[42] := delta.deltaPositionGyroscopeBiasJacobian_m_s[2, 1];
  row[43] := delta.deltaPositionGyroscopeBiasJacobian_m_s[2, 2];
  row[44] := delta.deltaPositionGyroscopeBiasJacobian_m_s[2, 3];
  row[45] := delta.deltaPositionGyroscopeBiasJacobian_m_s[3, 1];
  row[46] := delta.deltaPositionGyroscopeBiasJacobian_m_s[3, 2];
  row[47] := delta.deltaPositionGyroscopeBiasJacobian_m_s[3, 3];
  row[48] := delta.deltaPositionAccelerometerBiasJacobian_s2[1, 1];
  row[49] := delta.deltaPositionAccelerometerBiasJacobian_s2[1, 2];
  row[50] := delta.deltaPositionAccelerometerBiasJacobian_s2[1, 3];
  row[51] := delta.deltaPositionAccelerometerBiasJacobian_s2[2, 1];
  row[52] := delta.deltaPositionAccelerometerBiasJacobian_s2[2, 2];
  row[53] := delta.deltaPositionAccelerometerBiasJacobian_s2[2, 3];
  row[54] := delta.deltaPositionAccelerometerBiasJacobian_s2[3, 1];
  row[55] := delta.deltaPositionAccelerometerBiasJacobian_s2[3, 2];
  row[56] := delta.deltaPositionAccelerometerBiasJacobian_s2[3, 3];
end packDelta;
