within LieGroups.SE3;
package WithEulerB321
  extends LieGroups.SE3.Generic(
    redeclare package Rotation = LieGroups.SO3.EulerSequences.B321);
end WithEulerB321;
