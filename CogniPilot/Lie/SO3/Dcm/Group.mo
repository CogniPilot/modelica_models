within CogniPilot.Lie.SO3.Dcm;
operator record Group
  extends SO3.Base.Group;

  encapsulated operator function product
    import CogniPilot.Lie.SO3.Dcm;

    input Dcm.Element a,b;

    output Dcm.Element c;

  algorithm
    c := Dcm.Element(
      r=a.r*b.r);

  end product;

  encapsulated operator function inverse
    import CogniPilot.Lie.SO3.Dcm;

    input Dcm.Element a;

    output Dcm.Element res;

  algorithm
    res := Dcm.Element(
      r=transpose(a.r));

  end inverse;

  encapsulated operator function one
    import CogniPilot.Lie.SO3.Dcm;

    constant output Dcm.Element res;

  algorithm
    res := Dcm.Element(
      r=identity(3));

  end one;

  encapsulated operator function equal
    import CogniPilot.Lie.SO3.Dcm;

    input Dcm.Element a,b;

    input Real eps;

    output Boolean res;

  algorithm
    res := max(
      abs(
        a.r-b.r)) < eps;

  end equal;

  encapsulated operator function adjoint
    import CogniPilot.Lie.SO3.Dcm;

    input Dcm.Element a;

    output Real R[3,3];

  algorithm
    R := Dcm.to_matrix(
      a);

  end adjoint;

  encapsulated operator function exp
    import CogniPilot.Lie.SO3;

    input SO3.Algebra.Element a;

    output SO3.Dcm.Element res;

  protected
    Real x,C1,C2,X;

  algorithm
    X := a.to_matrix();

    x := sqrt(
      a.r*a.r);

    if(angle > 1e-7) then
      C1 := sin(
        x)/x;

      C2 :=(1-cos(
        x))/x^2;

    else
      C1 := 1-x^2/6+x^4/120;

      C2 := 1/2-x^2/24+x^4/720;

    end if;

    res := SO3.Dcm.Element(
      r=identity(3)+a*X+b*X*X);

  end exp;

  encapsulated operator function log
    import CogniPilot.Lie.SO3;

    input SO3.Dcm.Element a;

    output SO3.Algebra.Element res;

  protected
    Real x,c1;
  
  algorithm
    x := acos(
      ((a[1,1]+a[2,2]+a[3,3])-1)/2);
  
    if x > 1e-3 then
      c1 := x/(2*sin(x));
  
    else
      c1 := 1/2+x^2/12+7*x^4/720+31*x^6/30240;
  
    end if;
  
    res := c1*(a-transpose(a));
  end log;

  encapsulated operator function to_matrix
    import CogniPilot.Lie.SO3.Dcm;

    input Dcm.Element a;

    output Real R[3,3];

  algorithm
    R := a.r;

    annotation (
      Inline=true);

  end to_matrix;

  model Kinematics
    import CogniPilot.Lie.SO3.Dcm;

    input Dcm.Element a0;

    input Real[3] omega;

    output Dcm.Element a;

  initial equation
    a.r=a0.r;

  equation
    der(
      a.r)=a.r*skew(
      omega);

  end Kinematics;

end Group;