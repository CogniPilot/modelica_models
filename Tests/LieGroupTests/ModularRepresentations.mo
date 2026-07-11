within Tests.LieGroupTests;
model ModularRepresentations
  "Replaceable rotation representations produce equivalent SE(3) and SE_2(3) behavior"
  package SE3B232 = LieGroups.SE3.Generic(
    redeclare package Rotation = LieGroups.SO3.EulerSequences.B232);
  package SE23S123 = LieGroups.SE23.Generic(
    redeclare package Rotation = LieGroups.SO3.EulerSequences.S123);
  constant Real se3Tangent[6] = {0.4, -0.2, 0.3, 0.08, -0.04, 0.06};
  constant Real se23Tangent[9] = {
    0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04};
  LieGroups.SE3.WithQuaternion.Element se3Quaternion;
  LieGroups.SE3.WithMrp.Element se3Mrp;
  LieGroups.SE3.WithDcm.Element se3Dcm;
  LieGroups.SE3.WithEulerB321.Element se3Euler;
  SE3B232.Element se3B232;
  LieGroups.SE23.WithQuaternion.Element se23Quaternion;
  LieGroups.SE23.WithMrp.Element se23Mrp;
  LieGroups.SE23.WithDcm.Element se23Dcm;
  LieGroups.SE23.WithEulerB321.Element se23Euler;
  SE23S123.Element se23S123;
  LieGroups.SE23.WithQuaternion.Element mixedQuaternion;
  LieGroups.SE23.WithMrp.Element mixedMrp;
equation
  se3Quaternion = LieGroups.SE3.WithQuaternion.exp_map(se3Tangent);
  se3Mrp = LieGroups.SE3.WithMrp.exp_map(se3Tangent);
  se3Dcm = LieGroups.SE3.WithDcm.exp_map(se3Tangent);
  se3Euler = LieGroups.SE3.WithEulerB321.exp_map(se3Tangent);
  se3B232 = SE3B232.exp_map(se3Tangent);
  se23Quaternion = LieGroups.SE23.WithQuaternion.exp_map(se23Tangent);
  se23Mrp = LieGroups.SE23.WithMrp.exp_map(se23Tangent);
  se23Dcm = LieGroups.SE23.WithDcm.exp_map(se23Tangent);
  se23Euler = LieGroups.SE23.WithEulerB321.exp_map(se23Tangent);
  se23S123 = SE23S123.exp_map(se23Tangent);
  mixedQuaternion = LieGroups.SE23.WithQuaternion.exp_mixed(
    se23Quaternion, zeros(9), zeros(9), [0.0, 0.25; 0.0, 0.0]);
  mixedMrp = LieGroups.SE23.WithMrp.exp_mixed(
    se23Mrp, zeros(9), zeros(9), [0.0, 0.25; 0.0, 0.0]);

  assert(Tests.Assertions.maxAbsMatrix(
      LieGroups.SE3.WithQuaternion.to_Matrix(se3Quaternion)
        - LieGroups.SE3.WithMrp.to_Matrix(se3Mrp)) < 1.0e-8 and
         Tests.Assertions.maxAbsMatrix(
      LieGroups.SE3.WithQuaternion.to_Matrix(se3Quaternion)
        - LieGroups.SE3.WithDcm.to_Matrix(se3Dcm)) < 1.0e-8 and
         Tests.Assertions.maxAbsMatrix(
      LieGroups.SE3.WithQuaternion.to_Matrix(se3Quaternion)
        - LieGroups.SE3.WithEulerB321.to_Matrix(se3Euler)) < 1.0e-8 and
         Tests.Assertions.maxAbsMatrix(
      LieGroups.SE3.WithQuaternion.to_Matrix(se3Quaternion)
        - SE3B232.to_Matrix(se3B232)) < 1.0e-8,
    "Replaceable SE3 rotation representations disagreed");
  assert(Tests.Assertions.maxAbsVector(
      LieGroups.SE3.WithQuaternion.log_map(se3Quaternion) - se3Tangent) < 1.0e-8 and
         Tests.Assertions.maxAbsVector(
      LieGroups.SE3.WithMrp.log_map(se3Mrp) - se3Tangent) < 1.0e-8 and
         Tests.Assertions.maxAbsVector(
      LieGroups.SE3.WithDcm.log_map(se3Dcm) - se3Tangent) < 1.0e-8,
    "Replaceable SE3 exp/log representations disagreed");
  assert(Tests.Assertions.maxAbsMatrix(
      LieGroups.SE23.WithQuaternion.to_Matrix(se23Quaternion)
        - LieGroups.SE23.WithMrp.to_Matrix(se23Mrp)) < 1.0e-8 and
         Tests.Assertions.maxAbsMatrix(
      LieGroups.SE23.WithQuaternion.to_Matrix(se23Quaternion)
        - LieGroups.SE23.WithDcm.to_Matrix(se23Dcm)) < 1.0e-8 and
         Tests.Assertions.maxAbsMatrix(
      LieGroups.SE23.WithQuaternion.to_Matrix(se23Quaternion)
        - LieGroups.SE23.WithEulerB321.to_Matrix(se23Euler)) < 1.0e-8 and
         Tests.Assertions.maxAbsMatrix(
      LieGroups.SE23.WithQuaternion.to_Matrix(se23Quaternion)
        - SE23S123.to_Matrix(se23S123)) < 1.0e-8,
    "Replaceable SE_2(3) rotation representations disagreed");
  assert(Tests.Assertions.maxAbsMatrix(
      LieGroups.SE23.WithQuaternion.to_Matrix(mixedQuaternion)
        - LieGroups.SE23.WithMrp.to_Matrix(mixedMrp)) < 1.0e-8,
    "Replaceable SE_2(3) mixed exponential representations disagreed");
end ModularRepresentations;
