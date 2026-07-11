within LieGroups.SE3;
package WithQuaternion
  extends LieGroups.SE3.Generic(
    redeclare package Rotation = LieGroups.SO3.Representations.Quaternion);
end WithQuaternion;
