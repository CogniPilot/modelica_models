within LieGroups.SE3;
package WithDcm
  extends LieGroups.SE3.Generic(
    redeclare package Rotation = LieGroups.SO3.Representations.Dcm);
end WithDcm;
