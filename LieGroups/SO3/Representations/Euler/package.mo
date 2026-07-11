within LieGroups.SO3.Representations;
package Euler
  "Configurable Euler representation supporting all 24 sequence/convention forms"
  extends LieGroups.SO3.Interfaces.PartialRotation;
  constant LieGroups.SO3.Euler.Axis sequence[3] = {
    LieGroups.SO3.Euler.Axis.z,
    LieGroups.SO3.Euler.Axis.y,
    LieGroups.SO3.Euler.Axis.x};
  constant Boolean bodyFixed = true;

  redeclare record extends Orientation
    Real angles[3] "Euler angles in the configured sequence";
  end Orientation;

  redeclare function identity
    output Orientation element;
  algorithm
    element.angles := zeros(3);
  end identity;

  redeclare function product
    input Orientation left;
    input Orientation right;
    output Orientation result;
  algorithm
    result.angles := LieGroups.SO3.Euler.from_Matrix(
      LieGroups.SO3.Euler.to_Matrix(left.angles, sequence, bodyFixed)
        * LieGroups.SO3.Euler.to_Matrix(right.angles, sequence, bodyFixed),
      sequence,
      bodyFixed);
  end product;

  redeclare function inverse
    input Orientation element;
    output Orientation result;
  protected
    Real R[3, 3];
  algorithm
    R := LieGroups.SO3.Euler.to_Matrix(element.angles, sequence, bodyFixed);
    result.angles := LieGroups.SO3.Euler.from_Matrix(
      transpose(R), sequence, bodyFixed);
  end inverse;

  redeclare function exp_map
    input Real tangent[3];
    output Orientation element;
  algorithm
    element.angles := LieGroups.SO3.Euler.from_Matrix(
      LieGroups.SO3.Dcm.exp_map(tangent), sequence, bodyFixed);
  end exp_map;

  redeclare function log_map
    input Orientation element;
    output Real tangent[3];
  algorithm
    tangent := LieGroups.SO3.Dcm.log_map(
      LieGroups.SO3.Euler.to_Matrix(element.angles, sequence, bodyFixed));
  end log_map;

  redeclare function to_Matrix
    input Orientation element;
    output Real R[3, 3];
  algorithm
    R := LieGroups.SO3.Euler.to_Matrix(element.angles, sequence, bodyFixed);
  end to_Matrix;

  redeclare function from_Matrix
    input Real R[3, 3];
    output Orientation element;
  algorithm
    element.angles := LieGroups.SO3.Euler.from_Matrix(R, sequence, bodyFixed);
  end from_Matrix;
end Euler;
