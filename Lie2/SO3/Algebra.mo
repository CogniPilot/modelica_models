within Lie2.SO3;

model Algebra
    
  type Element = Real[3];
    
  type BaseType = Base.Algebra(
    redeclare type Element = Element);
  extends BaseType;
  
  function exp // TODO
    extends BaseType.exp;
  algorithm
    res := a;
    annotation(Inline = true);
  end exp;

  function bracket
    extends BaseType.bracket;
  algorithm
    res := {a[2]*b[3]-b[2]*a[3],a[3]*b[1]-b[3]*a[1],a[1]*b[2]-b[1]*a[2]};
    annotation(Inline = true);
  end bracket;

  function fromMatrix
    extends BaseType.fromMatrix;
  algorithm
    res := {a[3, 2], a[1, 3], a[2, 1]};
    annotation(Inline = true);
  end fromMatrix;

  function toMatrix
    extends BaseType.toMatrix;
  algorithm
    res := skew(a);
    annotation(Inline = true);
  end toMatrix;

end Algebra;