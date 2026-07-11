within LieGroups.SE23;
package Generic "SE_2(3) parameterized by a replaceable SO(3) representation"
  replaceable package Rotation = LieGroups.SO3.Representations.Quaternion
    constrainedby LieGroups.SO3.Interfaces.PartialRotation;
end Generic;
