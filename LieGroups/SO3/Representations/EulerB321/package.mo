within LieGroups.SO3.Representations;
package EulerB321 "Body-fixed Z-Y-X Euler representation"
  extends LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.z,
              LieGroups.SO3.Euler.Axis.y,
              LieGroups.SO3.Euler.Axis.x},
    bodyFixed=true);
end EulerB321;
