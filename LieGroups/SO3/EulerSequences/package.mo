within LieGroups.SO3;
package EulerSequences
  "Named intrinsic (B) and extrinsic (S) Euler representations; 1=x, 2=y, 3=z"
  package B121 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, bodyFixed=true);
  package B131 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x}, bodyFixed=true);
  package B212 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y}, bodyFixed=true);
  package B232 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y}, bodyFixed=true);
  package B313 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z}, bodyFixed=true);
  package B323 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z}, bodyFixed=true);
  package B123 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z}, bodyFixed=true);
  package B132 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y}, bodyFixed=true);
  package B213 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z}, bodyFixed=true);
  package B231 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x}, bodyFixed=true);
  package B312 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y}, bodyFixed=true);
  package B321 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, bodyFixed=true);

  package S121 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, bodyFixed=false);
  package S131 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x}, bodyFixed=false);
  package S212 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y}, bodyFixed=false);
  package S232 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y}, bodyFixed=false);
  package S313 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z}, bodyFixed=false);
  package S323 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z}, bodyFixed=false);
  package S123 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z}, bodyFixed=false);
  package S132 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y}, bodyFixed=false);
  package S213 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z}, bodyFixed=false);
  package S231 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x}, bodyFixed=false);
  package S312 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y}, bodyFixed=false);
  package S321 = LieGroups.SO3.Representations.Euler(
    sequence={LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, bodyFixed=false);
end EulerSequences;
