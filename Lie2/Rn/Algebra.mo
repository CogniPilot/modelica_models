within Lie2.Rn;

model Algebra
  
  type Element = Real[n];

  type BaseType = Base.Algebra(
    redeclare type Element = Element);
  extends BaseType;

  function add
    extends BaseType.add;
  algorithm
    res := a + b;
    annotation(Inline = true);
  end add;
  
  function exp
    extends BaseType.exp(redeclare type GroupElement=Group.Element);
  algorithm
    res := a;
    annotation(Inline = true);
  end exp;
  
end Algebra;