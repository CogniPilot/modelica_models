within Lie2.Rn;

model Algebra
  
  type Element = Real[n];

  type BaseType = Base.Algebra(
    redeclare type Element = Element);
  extends BaseType;

  function fromMatrix
    extends BaseType.fromMatrix;
  algorithm
    res := { a[i, n+1] for i in 1:n };
  end fromMatrix;

  function toMatrix
    extends BaseType.toMatrix;
  algorithm
    res := cat(2,
      cat(1, zeros(n, n), zeros(1, n)),
      cat(1, matrix(a), zeros(1, 1)));
    annotation(Inline = true);
  end toMatrix;

end Algebra;