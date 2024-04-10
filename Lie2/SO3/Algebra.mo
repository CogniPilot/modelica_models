within Lie2.SO3;

model Algebra
    
  record Element
    Real r[3];
  end Element;
    
  type BaseType = Base.Algebra(
    redeclare type ElementType = Element,
    redeclare type GroupElement = Group.Element);
  extends BaseType;

  function add
    extends BaseType.add;
  algorithm
    res := Element(a.r + b.r);
    annotation(Inline = true);
  end add;
  
  function exp
    extends BaseType.exp;
  algorithm
    res := Group.Element(a.r);
    annotation(Inline = true);
  end exp;
  
end Algebra;