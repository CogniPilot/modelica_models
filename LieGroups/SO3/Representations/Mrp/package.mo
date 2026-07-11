within LieGroups.SO3.Representations;
package Mrp
  extends LieGroups.SO3.Interfaces.PartialRotation;

  redeclare record extends Orientation
    Real r[3] "Modified Rodrigues parameters";
  end Orientation;

  redeclare function identity
    output Orientation element;
  algorithm
    element.r := zeros(3);
  end identity;

  redeclare function product
    input Orientation left;
    input Orientation right;
    output Orientation result;
  algorithm
    result.r := LieGroups.SO3.Mrp.product(left.r, right.r);
  end product;

  redeclare function inverse
    input Orientation element;
    output Orientation result;
  algorithm
    result.r := LieGroups.SO3.Mrp.inverse(element.r);
  end inverse;

  redeclare function exp_map
    input Real tangent[3];
    output Orientation element;
  algorithm
    element.r := LieGroups.SO3.Mrp.exp_map(tangent);
  end exp_map;

  redeclare function log_map
    input Orientation element;
    output Real tangent[3];
  algorithm
    tangent := LieGroups.SO3.Mrp.log_map(element.r);
  end log_map;

  redeclare function to_Matrix
    input Orientation element;
    output Real R[3, 3];
  algorithm
    R := LieGroups.SO3.Mrp.to_DCM(element.r);
  end to_Matrix;

  redeclare function from_Matrix
    input Real R[3, 3];
    output Orientation element;
  algorithm
    element.r := LieGroups.SO3.Mrp.from_Quat(LieGroups.SO3.Quat.from_DCM(R));
  end from_Matrix;
end Mrp;
