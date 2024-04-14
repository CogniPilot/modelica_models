within Lie2.SO3;

model Mrp
  type Element = Real[3];
  type BaseType = Base.Group(redeclare type Element = Element, redeclare type AlgebraElement = Algebra.Element);
  extends BaseType;

  function one
    extends BaseType.one;
  algorithm
    res := {0, 0, 0};
    annotation(
      Inline = true);
  end one;

  function product
    extends BaseType.product;
  protected
    Real na_sq, nb_sq, den, res[3];
  algorithm
    na_sq := a*a;
    nb_sq := b*b;
    den := 1 + na_sq*nb_sq - 2*a*b;
    if abs(den) > Constants.eps then
      res := ((1 - na_sq)*b + (1 - nb_sq)*a - 2*cross(b, a))/den;
    else
      res := {0, 0, 0};
    end if;
    annotation(
      Inline = true);
  end product;

  function inv
    extends BaseType.inv;
  algorithm
    res := -a;
    annotation(
      Inline = true);
  end inv;

  function log
    extends BaseType.log;
  protected
    Real n;
  algorithm
    res := -a;
    n := sqrt(a*a);
    if (n > 1e-7) then
      res := 4*atan(n)*a/n;
    else
      res := {0, 0, 0};
    end if;
    annotation(
      Inline = true);
  end log;

  function Dl
    extends BaseType.Dl;
  algorithm
    res := fromMatrix(Algebra.toMatrix(w)*toMatrix(a));
    annotation(
      Inline = true);
  end Dl;

  function Dr
    extends BaseType.Dr;
  protected
    Real n_sq;
  algorithm
    n_sq := a*a;
    res := 0.25*((1 - n_sq)*identity(3) + 2*skew(a) + 2*matrix(a)*transpose(matrix(a)))*w;
    annotation(
      Inline = true);
  end Dr;

  function normError
    extends BaseType.normError(redeclare constant output Real res = 0);
  algorithm
    res := 0;
    annotation(
      Inline = true);
  end normError;

  function normalize
    extends BaseType.normalize;
  algorithm
    res := a;
    annotation(
      Inline = true);
  end normalize;

  function switchCoordinates
    extends BaseType.switchCoordinates;
  protected
    Real n_sq;
  algorithm
    n_sq := a*a;
    res := -a/n_sq;
  end switchCoordinates;

  function shouldSwitchCoordinates
    extends BaseType.shouldSwitchCoordinates;
  algorithm
    res := a*a > 1;
  end shouldSwitchCoordinates;

  function fromQuat
    input Quat.Element a;
    output Element res;
  algorithm
    if abs(a[1]) < Constants.eps then
      res := {0, 0, 0};
    else
      res := a[2:4]/(1 + a[1]);
    end if;
    if shouldSwitchCoordinates(res) then
      res := switchCoordinates(res);
    end if;
    annotation(
      Inline = true);
  end fromQuat;

  function equal
    extends BaseType.equal;
  algorithm
    res := Math.norm2(log(product(a, inv(b)))) < eps;
    annotation(Inline = true);
  end equal;

  function fromMatrix
    extends BaseType.fromMatrix;
  algorithm
    res := fromQuat(SO3.Quat.fromMatrix(a));
    annotation(
      Inline = true);
  end fromMatrix;

  function toMatrix
    extends BaseType.toMatrix;
  protected
    Real X[3, 3], n_sq;
  algorithm
    X := skew(a);
    n_sq := a*a;
    res := identity(3) + (8*X*X + 4*(1 - n_sq)*X)/(1 + n_sq)^2;
    annotation(
      Inline = true);
  end toMatrix;
end Mrp;