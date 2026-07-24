within;
package RnProjection
  model Product
    constant Real x[3] = {1, 2, 3};
    constant Real y[3] = {4, 5, 6};
    Real result[3];
  equation
    result = LieGroups.Rn.product(x, y);
  end Product;

  model ToMatrix
    constant Real x[3] = {1, 2, 3};
    Real result[4, 4];
  equation
    result = LieGroups.Rn.to_Matrix(x);
  end ToMatrix;

  model Wedge
    constant Real x[3] = {1, 2, 3};
    Real result[4, 4];
  equation
    result = LieGroups.Rn.wedge(x);
  end Wedge;

  model Vee
    constant Real X[4, 4] = identity(4);
    Real result[3];
  equation
    result = LieGroups.Rn.vee(X);
  end Vee;

  model Adjoint
    constant Real x[3] = {1, 2, 3};
    Real result[3, 3];
  equation
    result = LieGroups.Rn.adjoint(x);
  end Adjoint;
end RnProjection;
