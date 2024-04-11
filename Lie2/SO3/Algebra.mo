within Lie2.SO3;

model Algebra
    
  type Element = Real[3];
    
  type BaseType = Base.Algebra(
    redeclare type Element = Element,
    redeclare type GroupElement = Group.Element);
  extends BaseType;

  function add
    extends BaseType.add;
  algorithm
    res := Element(r=a.r + b.r);
    annotation(Inline = true);
  end add;
  
  function exp
    extends BaseType.exp;
  algorithm
    res := a;
    annotation(Inline = true);
  end exp;
  
end Algebra;