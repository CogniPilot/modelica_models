within LieGroups.SE23;
package WithQuaternion
  extends LieGroups.SE23.Generic(
    redeclare package Rotation = LieGroups.SO3.Representations.Quaternion);
end WithQuaternion;
