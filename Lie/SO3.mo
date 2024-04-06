within Lie;

package SO3

  operator record Algebra
  
    extends Lie.Algebra;
  
    Real r[3];
    
    encapsulated operator '+'
      import Lie.SO3.Algebra;

      function add
        input Algebra a;
        input Algebra b;
        output Algebra res;
      algorithm
        res.r := a.r + b.r;
        annotation(Inline = true);
      end add;
    
    end '+';
    
    function leftJac
      input Algebra phi;
      output Real[3, 3] res;
    protected
      Real x, c1, c2;
    algorithm
      x := sqrt(phi.r * phi.r);
      if x > 1e-3 then
        c1 := (1 - cos(x))/x^2;
        c2 := (x - sin(x))/x^3;
      else
        c1 := 1/2 - x^2/24 + x^4/720;
        c2 := 1/6 - x^2/120 + x^4/50540;
      end if;
      res := identity(3) + c1 * skew(phi.r) + c2 * skew(phi.r) * skew(phi.r);
      annotation(Inline = true);
    end leftJac;

    function leftJacInv
      input Algebra phi;
      output Real[3, 3] res;
    protected
      Real x, c1;
    algorithm
      x := sqrt(phi.r * phi.r);
      if x > 1e-3 then
        c1 := (2 - x/tan(x/2))/(2*x^2);
      else
        c1 := 1/12 + x^2/720 + x^4/30240;
      end if;
      res := identity(3) - 1/2 * skew(phi.r) + c1 * skew(phi.r) * skew(phi.r);
      annotation(Inline = true);
    end leftJacInv;

    function rightJac
      input Algebra phi;
      output Real[3, 3] res;
    protected
      Real x, c1, c2;
    algorithm
      x := sqrt(phi.r * phi.r);
      if x > 1e-3 then
        c1 := (1 - cos(x))/x^2;
        c2 := (x - sin(x))/x^3;
      else
        c1 := (1 - cos(x))/x^2;
        c2 := 1/6 - x^2/120 + x^4/50540;
      end if;
      res := identity(3) - c1 * skew(phi.r) + c2 * skew(phi.r) * skew(phi.r);
      annotation(Inline = true);
    end rightJac;

    function rightJacInv
      input Algebra phi;
      output Real[3, 3] res;
    protected
      Real x, c1;
    algorithm
      x := sqrt(phi.r * phi.r);
      if x > 1e-3 then
        c1 := (2 - x/tan(x/2))/(2*x^2);
      else
        c1 := 1/12 + x^2/720 + x^4/30240;
      end if;
      res := identity(3) + 1/2 * skew(phi.r) + c1 * skew(phi.r) * skew(phi.r);
      annotation(Inline = true);
    end rightJacInv;

    function expDcm
      import Lie.SO3.Dcm;
      input Algebra phi;
      output Dcm res;
    protected
      Real x, c1, c2;
    algorithm
      x := sqrt(phi.r * phi.r);
      if x > 1e-3 then
        c1 := sin(x)/x;
        c2 := (1 - cos(x))/x^2;
      else
        c1 := 1 - x^2/6 + x^4/120;
        c2 := 1/2 - x^2/24 + x^4/720;
      end if;
      res.r := identity(3) + c1 * skew(phi.r) + c2 * skew(phi.r) * skew(phi.r);
      annotation(Inline = true);
    end expDcm;
  
    function fromMatrix "algebra element from matrix"
      input Real[3, 3] a;
      output Algebra res;
    algorithm
      res.r := {a[3, 2], a[1, 3], a[2, 1]};
      annotation(Inline = true);
    end fromMatrix;
  
    function toMatrix "matrix to algebra element"
      input Algebra a;
      output Real[3, 3] res;
    algorithm
      res := skew(a.r);
      annotation(Inline = true);
    end toMatrix;
    
  end Algebra;
   
  operator record Dcm
  
    extends Lie.Group;
    
    Real r[3, 3];
    
    encapsulated operator '*'
      import Lie.SO3.Dcm;
      import Lie.SO3.Algebra;

      function product
        input Dcm a;
        input Dcm b;
        output Dcm res;
      algorithm
        res.r := a.r * b.r;
        annotation(Inline = true);
      end product;
    
    end '*';
  
    function inv
      input Dcm a;
      output Dcm res;
    algorithm
      res.r := transpose(a.r);
      annotation(Inline = true);
    end inv;
  
    function one
      constant output Dcm res;
    algorithm
      res.r := identity(3);
      annotation(Inline = true);
    end one;
    
    function deriv
      input Dcm a;
      input Real[3] omega;
      input Lie.TangentSpace tangentSpace;
      output Real[3, 3] r_dot;
    algorithm
      if (tangentSpace == TangentSpace.Right) then
        r_dot := a.r * skew(omega);
      elseif (tangentSpace == TangentSpace.Left) then
        r_dot := skew(omega) * a.r;
      end if;
      annotation(Inline = true);
    end deriv;
    
    function log
      input Dcm a;
      output Algebra res;
    protected
      Real x, c1;
    algorithm
      x := acos(((a.r[1, 1] + a.r[2, 2] + a.r[3, 3]) - 1)/2);
      if x > 1e-3 then
        c1 := x/(2*sin(x));
      else
        c1 := 1/2 + x^2/12 + 7*x^4/720 + 31*x^6/30240;
      end if;
      res := Algebra.fromMatrix(c1 * (a.r - transpose(a.r)));
      annotation(Inline = true);
    end log;
  
    function fromMatrix "group element from matrix"
      input Real[3, 3] a;
      output Dcm res;
    algorithm
      res.r := a;
      annotation(Inline = true);
    end fromMatrix;
  
    function toMatrix "group element to matrix"
      input Dcm a;
      output Real[3, 3] res;
    algorithm
      res := a.r;
      annotation(Inline = true);
    end toMatrix;
  
    function fromEuler "group element from euler sequence"
      input Lie.SO3.Euler a;
      output Dcm res;
    algorithm
      res := Dcm.fromMatrix(Lie.SO3.Euler.toMatrix(a));
      annotation(Inline = true);
    end fromEuler;
  
    function orthoNormalize
      // see Barfoot 2024, pg. 303
      // NOTE: using Modelica.Math here makes it difficult to use Casadi
      input Real[3, 3] a;
      output Real[3, 3] res;
    algorithm
      res := Modelica.Math.Matrices.inv(
        Modelica.Math.Matrices.cholesky(a*transpose(a)))*a;
      annotation(Inline = true);
    end orthoNormalize;
  
    function orthoNormError
      input Dcm a;
      output Real error;
    protected
      Real x[3], y[3], z[3];
    algorithm
      x := a.r[1, :];
      y := a.r[2, :];
      z := a.r[3, :];
      error := max(abs({x*y, x*z, y*z, 1 - x*x, 1 - y*y, 1 - z*z}));
    end orthoNormError;
  
    model Kinematics
      constant input Dcm C0;
      input Real[3] omega;
      parameter Lie.TangentSpace tangentSpace;
      output Dcm C;
      output Real orthoNormError;
      parameter Real tol = 1e-4;
    initial equation
      C.r = C0.r;
    equation
      der(C.r) = Lie.SO3.Dcm.deriv(C, omega, tangentSpace);
      orthoNormError = Lie.SO3.Dcm.orthoNormError(C);
      when (orthoNormError > tol) then
        reinit(C.r, Lie.SO3.Dcm.orthoNormalize(pre(C.r)));
      end when;
    end Kinematics;
  
  end Dcm;


    operator record Euler
    
    type RotationType = enumeration(Body "Body", Space "Space");
  
      extends Lie.Group;
      
      Real angles[3];
      parameter Integer axisSeq[3];
      parameter RotationType rotType;
    
      function toMatrix "Euler Sequence to Matrix"
        input Euler a;
        output Real[3, 3] res;
      algorithm
        res := identity(3);
        for i in 1:3 loop
          if (a.rotType == RotationType.Space) then
            res := rotationMatrix(a.angles[i], a.axisSeq[i]) * res;
          elseif (a.rotType == RotationType.Body) then
            res := res * rotationMatrix(a.angles[i], a.axisSeq[i]);
          end if;
        end for;
        annotation(Inline = true);
      end toMatrix;
    
  
    function rotationMatrix
      input Real x;
      input Integer axis;
      output Real[3, 3] res;
    algorithm
      assert(axis > 0 and axis < 4, "axis out of range");
      if (axis == 1) then
        res := {
          {1, 0, 0},
          {0, cos(x), -sin(x)},
          {0, sin(x), cos(x)}};
      elseif (axis == 2) then
        res := {
          {cos(x), 0, sin(x)},
          {0, 1, 0},
          {-sin(x), 0, cos(x)}};
      elseif (axis == 3) then
        res := {
          {cos(x), -sin(x), 0},
          {sin(x), cos(x), 0},
          {0, 0, 1}};
      end if;
      annotation(Inline = true);
    end rotationMatrix;
    
  end Euler;
end SO3;