within LieGroups.SO3.Interfaces;
partial package PartialRotation
  "Contract implemented by replaceable SO(3) coordinate representations"
  replaceable record Orientation
    Real interfaceMarker[0] "Empty marker retained by redeclared records";
  end Orientation;

  replaceable partial function identity
    output Orientation element;
  end identity;

  replaceable partial function product
    input Orientation left;
    input Orientation right;
    output Orientation result;
  end product;

  replaceable partial function inverse
    input Orientation element;
    output Orientation result;
  end inverse;

  replaceable partial function exp_map
    input Real tangent[3];
    output Orientation element;
  end exp_map;

  replaceable partial function log_map
    input Orientation element;
    output Real tangent[3];
  end log_map;

  replaceable partial function to_Matrix
    input Orientation element;
    output Real R[3, 3];
  end to_Matrix;

  replaceable partial function from_Matrix
    input Real R[3, 3];
    output Orientation element;
  end from_Matrix;

  function leftInvariantError
    "Log(reference^(-1) actual), expressed in the reference frame"
    input Orientation reference;
    input Orientation actual;
    output Real tangent[3];
  protected
    Orientation relative;
  algorithm
    relative := product(inverse(reference), actual);
    tangent := log_map(relative);
  end leftInvariantError;

  function rightInvariantError
    "Log(actual reference^(-1)), expressed in the spatial frame"
    input Orientation reference;
    input Orientation actual;
    output Real tangent[3];
  protected
    Orientation relative;
  algorithm
    relative := product(actual, inverse(reference));
    tangent := log_map(relative);
  end rightInvariantError;
end PartialRotation;
