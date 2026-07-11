within LieGroups.SE3;
package WithMrp
  extends LieGroups.SE3.Generic(
    redeclare package Rotation = LieGroups.SO3.Representations.Mrp);
end WithMrp;
