within LieGroups.SO3.Representations;
package Quaternion
  extends LieGroups.SO3.Interfaces.PartialRotation;

  redeclare record extends Orientation
    Real q[4] "Scalar-first Hamilton quaternion";
  end Orientation;

  redeclare function identity
    output Orientation element;
  algorithm
    element.q := {1.0, 0.0, 0.0, 0.0};
  end identity;

  redeclare function product
    input Orientation left;
    input Orientation right;
    output Orientation result;
  algorithm
    result.q := LieGroups.SO3.Quat.product(left.q, right.q);
  end product;

  redeclare function inverse
    input Orientation element;
    output Orientation result;
  algorithm
    result.q := LieGroups.SO3.Quat.inverse(element.q);
  end inverse;

  redeclare function exp_map
    input Real tangent[3];
    output Orientation element;
  algorithm
    element.q := LieGroups.SO3.Quat.exp_map(tangent);
  end exp_map;

  redeclare function log_map
    input Orientation element;
    output Real tangent[3];
  algorithm
    tangent := LieGroups.SO3.Quat.log_map(element.q);
  end log_map;

  redeclare function to_Matrix
    input Orientation element;
    output Real R[3, 3];
  algorithm
    R := LieGroups.SO3.Quat.to_DCM(element.q);
  end to_Matrix;

  redeclare function from_Matrix
    input Real R[3, 3];
    output Orientation element;
  algorithm
    element.q := LieGroups.SO3.Quat.from_DCM(R);
  end from_Matrix;
end Quaternion;
