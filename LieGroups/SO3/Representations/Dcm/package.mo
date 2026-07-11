within LieGroups.SO3.Representations;
package Dcm
  extends LieGroups.SO3.Interfaces.PartialRotation;

  redeclare record extends Orientation
    Real R[3, 3] "Direction cosine matrix";
  end Orientation;

  redeclare function identity
    output Orientation element;
  algorithm
    element.R := identity(3);
  end identity;

  redeclare function product
    input Orientation left;
    input Orientation right;
    output Orientation result;
  algorithm
    result.R := left.R * right.R;
  end product;

  redeclare function inverse
    input Orientation element;
    output Orientation result;
  algorithm
    result.R := transpose(element.R);
  end inverse;

  redeclare function exp_map
    input Real tangent[3];
    output Orientation element;
  algorithm
    element.R := LieGroups.SO3.Dcm.exp_map(tangent);
  end exp_map;

  redeclare function log_map
    input Orientation element;
    output Real tangent[3];
  algorithm
    tangent := LieGroups.SO3.Dcm.log_map(element.R);
  end log_map;

  redeclare function to_Matrix
    input Orientation element;
    output Real R[3, 3];
  algorithm
    R := element.R;
  end to_Matrix;

  redeclare function from_Matrix
    input Real R[3, 3];
    output Orientation element;
  algorithm
    element.R := R;
  end from_Matrix;
end Dcm;
