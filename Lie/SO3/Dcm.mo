within Lie.SO3;
operator record Dcm
  extends Lie.Group;

  Real r[3,3];

  encapsulated operator '*'
    import Lie.SO3.Dcm;

    import Lie.SO3.Algebra;

    function product
      input Dcm a;

      input Dcm b;

      output Dcm res;

    algorithm
      res.r := a.r*b.r;

      annotation (
        Inline=true);

    end product;

  end '*';

  function inv
    input Dcm a;

    output Dcm res;

  algorithm
    res.r := transpose(
      a.r);

    annotation (
      Inline=true);

  end inv;

  function one
    constant output Dcm res;

  algorithm
    res.r := identity(
      3);

    annotation (
      Inline=true);

  end one;

  function deriv
    input Dcm a;

    input Real[3] omega;

    input Lie.TangentSpace tangentSpace;

    output Real[3,3] r_dot;

  algorithm
    if(tangentSpace == TangentSpace.Right) then
      r_dot := a.r*skew(
        omega);

    elseif(tangentSpace == TangentSpace.Left) then
      r_dot := skew(
        omega)*a.r;

    end if;

    annotation (
      Inline=true);

  end deriv;

  function log
    input Dcm a;
  
    output Algebra res;
  
  protected
    Real x,c1;
  
  algorithm
    x := acos(
      ((a.r[1,1]+a.r[2,2]+a.r[3,3])-1)/2);
  
    if x > 1e-3 then
      c1 := x/(2*sin(
        x));
  
    else
      c1 := 1/2+x^2/12+7*x^4/720+31*x^6/30240;
  
    end if;
  
    res := Algebra.fromMatrix(
      c1*(a.r-transpose(
        a.r)));
  
    annotation (
      Inline=true);
  
  end log;

  function fromMatrix
    "group element from matrix"
    input Real[3,3] a;

    output Dcm res;

  algorithm
    res.r := a;

    annotation (
      Inline=true);

  end fromMatrix;

  function toMatrix
    "group element to matrix"
    input Dcm a;

    output Real[3,3] res;

  algorithm
    res := a.r;

    annotation (
      Inline=true);

  end toMatrix;

  function fromEuler
    "group element from euler sequence"
    input Lie.SO3.Euler a;

    output Dcm res;

  algorithm
    res := Dcm.fromMatrix(
      Lie.SO3.Euler.toMatrix(
        a));

    annotation (
      Inline=true);

  end fromEuler;

  function orthoNormalize
    // see Barfoot 2024, pg. 303
    // NOTE: using Modelica.Math here makes it difficult to use Casadi
    input Real[3,3] a;

    output Real[3,3] res;

  algorithm
    res := Modelica.Math.Matrices.inv(
      Modelica.Math.Matrices.cholesky(
        a*transpose(
          a)))*a;

    annotation (
      Inline=true);

  end orthoNormalize;

  function orthoNormError
    input Dcm a;

    output Real error;

  protected
    Real x[3],y[3],z[3];

  algorithm
    x := a.r[1,:];

    y := a.r[2,:];

    z := a.r[3,:];

    error := max(
      abs(
        {x*y,x*z,y*z,1-x*x,1-y*y,1-z*z}));

  end orthoNormError;

  model Kinematics
    constant input Dcm C0;

    input Real[3] omega;

    parameter Lie.TangentSpace tangentSpace;

    output Dcm C;

    output Real orthoNormError;

    parameter Real tol=1e-4;

  initial equation
    C.r=C0.r;

  equation
    der(
      C.r)=Lie.SO3.Dcm.deriv(
      C,
      omega,
      tangentSpace);

    orthoNormError=Lie.SO3.Dcm.orthoNormError(
      C);

    when(orthoNormError > tol) then
      reinit(
        C.r,
        Lie.SO3.Dcm.orthoNormalize(
          pre(
            C.r)));

    end when;

  end Kinematics;

end Dcm;