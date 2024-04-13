within Lie2.SO3;

model Quat

  type Element = Real[4];
    
  type BaseType = Base.Group(
    redeclare type Element = Element,
    redeclare type AlgebraElement = Algebra.Element);
  
  extends BaseType;

  function one
    extends BaseType.one;
  algorithm
    res := {1, 0, 0, 0};
    annotation(Inline = true);
  end one;
   
  function product
    extends BaseType.product;
  algorithm
    res := {
      a[1] * b[1] - a[2] * b[2] - a[3] * b[3] - a[4] * b[4],
      a[2] * b[1] + a[1] * b[2] - a[4] * b[3] + a[3] * b[4],
      a[3] * b[1] + a[4] * b[2] + a[1] * b[3] - a[2] * b[4],
      a[4] * b[1] - a[3] * b[2] + a[2] * b[3] + a[1] * b[4]};
    annotation(Inline = true);
  end product;
  
  function inv
    extends BaseType.inv;
  algorithm
    res := {a[1], -a[2], -a[3], -a[4]};
    annotation(Inline = true);
  end inv;

  function log // TODO
    extends BaseType.log;
  protected
    Real n;
  algorithm
    res := -a;
    n := sqrt(a*a);
    if(n > 1e-7) then
      res := 4*atan(n)*a/n;
    else
      res := {0,0,0};
    end if;
    annotation(Inline = true);
  end log;
  
  function Dr
    extends BaseType.Dr;
  algorithm
    res := 0.5 * product(a, {0, w[1], w[2], w[3]});
    annotation(Inline = true);
  end Dr;

  function normError
    extends BaseType.normError;
  algorithm
    res := abs(1 - sqrt(a*a));
    annotation(Inline = true);
  end normError;

  function normalize
    extends BaseType.normalize;
  protected
    Real n;
  algorithm
    n := sqrt(a*a);
    if (n > Constants.eps) then
      res := a/n;
    else
      res := one();
    end if;
    annotation(Inline = true);
  end normalize;

  function toMatrix
    extends BaseType.toMatrix;
  protected
    Real aa, ab, ac, ad, bb, bc, bd, cc, cd, dd;
  algorithm
    aa := a[1]*a[1];
    ab := a[1]*a[2];
    ac := a[1]*a[3];
    ad := a[1]*a[4];
    bb := a[2]*a[2];
    bc := a[2]*a[3];
    bd := a[2]*a[4];
    cc := a[3]*a[3];
    cd := a[3]*a[4];
    dd := a[4]*a[4];
    res := {
      {aa + bb - cc - dd, 2 * (bc - ad), 2 * (bd + ac)},
      {2 * (bc + ad), aa + cc - bb - dd, 2 * (cd - ab)},
      {2 * (bd - ac), 2 * (cd + ab), aa + dd - bb - cc}
    };
    annotation(Inline = true);
  end toMatrix;

  function fromMatrix
    extends BaseType.fromMatrix;
  protected
    Real q_sq[4], q_max;
    Integer i_max;
  algorithm
    q_sq := {
      1 + a[1, 1] + a[2, 2] + a[3, 3], // scalar part
      1 - a[1, 1] + a[2, 2] - a[3, 3],
      1 - a[1, 1] - a[2, 2] + a[3, 3],
      1 + a[1, 1] + a[2, 2] + a[3, 3]}/4;
    i_max := 1;
    for i in 2:4 loop
      if q_sq[i] > q_sq[i_max] then
        i_max := i;
      end if;
    end for;
    assert(q_sq[i_max] > 0, "quaternion check failed");
    q_max := sqrt(q_sq[i_max]);
    if (i_max == 1) then
      res := {4*q_sq[i_max],
            a[3, 2] - a[2, 3], 
            a[1, 3] - a[3, 1], 
            a[2, 1] - a[1, 2]} / (4 * q_max);
    elseif (i_max == 2) then
      res := {a[3, 2] - a[2, 3], 
            4*q_sq[i_max],
            a[1, 2] - a[2, 1],
            a[1, 3] - a[3, 1]} / (4 * q_max);
    elseif (i_max == 3) then
      res := {a[1, 3] - a[3, 1],
            a[1, 2] - a[2, 1],
            4*q_sq[i_max],
            a[2, 3] - a[3, 2]} / (4 * q_max);
    else
      res := {a[2, 1] - a[1, 2],
            a[1, 3] - a[3, 1],
            a[2, 3] - a[3, 2], 
            4*q_sq[i_max]} / (4 * q_max);
    end if;
    annotation(Inline = true);
  end fromMatrix;

  function fromMrp
    input Mrp.Element a;
    output Element res;
  protected
    Real n_sq, den;
  algorithm
    n_sq := a * a;
    den := 1 + n_sq;
    res := {(1 - n_sq)/den, 2*a[1]/den, 2*a[2]/den, 2*a[3]/den};
    annotation(Inline = true);
  end fromMrp;

end Quat;