within LieGroups.SE3;
package Generic "SE(3) parameterized by a replaceable SO(3) representation"
  replaceable package Rotation = LieGroups.SO3.Representations.Quaternion
    constrainedby LieGroups.SO3.Interfaces.PartialRotation;
end Generic;
