within CogniPilot;
package Lie
  model Test
    Real omega[3]={1,2,3};
    Real R[3,3];
    CogniPilot.Lie.SO3.Mrp.Kinematics mrp(
      a0=SO3.Mrp.Element(
        r={0,0,0}),
      omega=omega);
  equation
    R=mrp.a.to_matrix();
  end Test;
  package SO3
    operator record Mrp
      extends SO3.Group;
      encapsulated operator record Element
        import CogniPilot.Lie;
        extends Lie.Group.Element;
        Real r[3];
        encapsulated operator '*'
          function product
            input Mrp.Element a,b;
            output Mrp.Element res;
          algorithm
            res := Mrp.product(
              a,
              b);
          end product;
        end '*';
        encapsulated operator function Ad=Lie.SO3.Mrp.adjoint(
          a=Lie.SO3.Mrp.Element(
            r));
        encapsulated operator function log=Lie.SO3.Mrp.log(
          a=Lie.SO3.Mrp.Element(
            r));
        encapsulated operator function to_matrix=Lie.SO3.Mrp.to_matrix(
          a=Lie.SO3.Mrp.Element(
            r));
      end Element;
      encapsulated operator function product
        input Mrp.Element a,b;
        output Mrp.Element c;
      protected
        Real na_sq,nb_sq,den,res[3];
      algorithm
        na_sq := a.r*a.r;
        nb_sq := b.r*b.r;
        den := 1+na_sq+nb_sq-2*a.r*b.r;
        c := Mrp.Element(
          r=((1-na_sq)*b.r+(1-nb_sq)*a.r-2*cross(b.r,a.r))/den);
      end product;
      encapsulated operator function inverse
        input Mrp.Element r1;
        output Mrp.Element res;
      algorithm
        res := Mrp(
          r=-r1.r);
      end inverse;
      encapsulated operator function one
        output Mrp.Element res;
      algorithm
        res := Mrp.Element(
          r={0,0,0});
      end one;
      encapsulated operator function shadow_if_necessary
        input Mrp.Element r1;
        output Mrp.Element res;
      protected
        Real n_sq;
      algorithm
        n_sq := r1.r*r1.r;
        if(n_sq > 1) then
          res := Mrp.Element(
            r=-r1.r/n_sq);
        else
          res := r1;
        end if;
      end shadow_if_necessary;
      encapsulated operator function adjoint
        input Mrp.Element a;
        output Real R[3,3];
      algorithm
        R := Mrp.to_matrix(
          a);
      end adjoint;
      encapsulated operator function exp
        input SO3.Algebra.Element a;
        output SO3.Mrp.Element res;
      protected
        Real angle;
      algorithm
        angle := sqrt(
          a.r*a.r);
        if(angle > 1e-7) then
          res := SO3.Mrp.shadow_if_necessary(
            SO3.Mrp.Element(
              r=tan(angle/4)*a.r/angle));
        else
          res := SO3.Mrp.Element(
            r={0,0,0});
        end if;
      end exp;
      encapsulated operator function log
        input SO3.Mrp.Element a;
        output SO3.Algebra.Element res;
      protected
        Real n;
      algorithm
        n := sqrt(
          a.r*a.r);
        if(n > 1e-7) then
          res := SO3.Algebra.Element(
            r=4*atan(n)*a.r/n);
        else
          res := SO3.Algebra.Element(
            r={0,0,0});
        end if;
      end log;
      encapsulated operator function to_matrix
        import CogniPilot.Lie.SO3.Mrp;
        input Mrp.Element a;
        output Real R[3,3];
      protected
        Real X[3,3],n_sq;
      algorithm
        X := skew(
          a.r);
        n_sq := a.r*a.r;
        R := identity(
          3)+(8*X*X+4*(1-n_sq)*X)/(1+n_sq)^2;
        annotation (
          Inline=false);
      end to_matrix;
      model Kinematics
        import CogniPilot.Lie.SO3.Mrp;
        input Mrp.Element a0;
        input Real[3] omega;
        output Mrp.Element a;
      protected
        Real v[3,1];
        Real n_sq;
        Real B[3,3];
      initial equation
        a.r=a0.r;
      equation
        v[1,1]=a.r[1];
        v[2,1]=a.r[2];
        v[3,1]=a.r[3];
        n_sq=a.r*a.r;
        B=0.25*((1-n_sq)*identity(
          3)+2*skew(
          a.r)+2*v*transpose(
          v));
        when(a.r*a.r > 1) then
          reinit(
            a.r,
            -a.r/(a.r*a.r));
        end when;
        der(
          a.r)=B*omega;
      end Kinematics;
    end Mrp;
    encapsulated operator record Algebra
      import CogniPilot.Lie;
      extends Lie.Algebra;
      encapsulated operator record Element
        import CogniPilot.Lie;
        extends Lie.Group.Element;
        Real r[3];
        encapsulated operator '+'
          import CogniPilot.Lie.SO3.Algebra;
          function addition
            input Algebra.Element a,b;
            output Algebra.Element res;
          algorithm
            res := Algebra.addition(
              a,
              b);
          end addition;
        end '+';
        encapsulated operator '*'
          import CogniPilot.Lie.SO3.Algebra;
          function bracket
            input Algebra.Element a,b;
            output Algebra.Element res;
          algorithm
            res := Algebra.bracket(
              a,
              b);
          end bracket;
          function scalar_multiply
            import CogniPilot.Lie.SO3.Algebra;
            input Real a;
            input Algebra.Element b;
            output Algebra.Element res;
          algorithm
            res := Algebra.Element(
              r=a*b.r);
          end scalar_multiply;
        end '*';
        encapsulated operator function exp
          import CogniPilot.Lie.SO3;
          input SO3.Algebra a;
          input SO3.Group group;
          output SO3.Group.Element res;
        algorithm
          res := group.exp(
            a);
        end exp;
        encapsulated operator function ad=Lie.SO3.Algebra.adjoint(
          a=Lie.SO3.Algebra.Element(
            r));
        encapsulated operator function vee=Lie.SO3.Algebra.vee(
          a=Lie.SO3.Algebra.Element(
            r));
      end Element;
      encapsulated operator function bracket
        import CogniPilot.Lie.SO3.Algebra;
        input Algebra.Element a,b;
        output Algebra.Element c;
      algorithm
        c := Algebra.Element(
          r={a.r[2]*b.r[3]-b.r[2]*a.r[3],a.r[3]*b.r[1]-b.r[3]*a.r[1],a.r[1]*b.r[2]-b.r[1]*a.r[2]});
      end bracket;
      encapsulated operator function addition
        import CogniPilot.Lie.SO3.Algebra;
        input Algebra.Element a,b;
        output Algebra.Element c;
      algorithm
        c := Algebra.Element(
          r=a.r+b.r);
      end addition;
      encapsulated operator function scalar_multiplication
        import CogniPilot.Lie.SO3.Algebra;
        input Real a;
        input Algebra.Element b;
        output Algebra.Element c;
      algorithm
        c := Algebra.Element(
          r=a*b.r);
      end scalar_multiplication;
      encapsulated operator function adjoint
        import CogniPilot.Lie.SO3.Algebra;
        input Real a;
        output Algebra.Element c;
      algorithm
        c := Algebra.to_matrix(
          a);
      end adjoint;
      encapsulated operator function wedge
        import CogniPilot.Lie.SO3.Algebra;
        input Real a[3];
        output Algebra.Element b;
      algorithm
        b := Algebra.Element(
          r=a);
      end wedge;
      encapsulated operator function vee
        import CogniPilot.Lie.SO3.Algebra;
        input Algebra.Element a;
        output Real b[3];
      algorithm
        b := a.r;
      end vee;
      encapsulated operator function to_matrix
        import CogniPilot.Lie.SO3.Algebra;
        input Algebra.Element a;
        output Real M[3,3];
      algorithm
        M[1,1] :=-a.r[3];
        M[2,1] := a.r[3];
        M[1,3] := a.r[2];
        M[3,1] :=-a.r[2];
        M[2,3] :=-a.r[1];
        M[3,2] := a.r[1];
      end to_matrix;
      encapsulated operator function from_matrix
        import CogniPilot.Lie.SO3.Algebra;
        input Real M[3,3];
        output Algebra.Element a;
      algorithm
        a := Algebra.Element(
          r={M[3,2],M[1,3],M[2,1]});
      end from_matrix;
    end Algebra;
    encapsulated operator record Group
      import CogniPilot.Lie;
      extends Lie.Group;
    end Group;
  end SO3;
  encapsulated operator record Group
    encapsulated operator record Element
    end Element;
  end Group;
  operator record Algebra
    encapsulated operator record Element
    end Element;
  end Algebra;
end Lie;
