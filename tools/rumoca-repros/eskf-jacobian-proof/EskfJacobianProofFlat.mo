within;
package EskfJacobianProofFlat
  "Flattened barometer retraction as a single self-contained function.

   EskfJacobianProof.retract_pos_z factors the barometer measurement through
   se23_exp and se23_product, so its synthesized expansion is a function that
   calls two others, is called from a third, and returns an array. The rumoca
   Solve IR declines that expansion in-process (EL005), so EskfJacobianProof is
   evaluated under OpenModelica. This flattened form inlines the same SE_2(3)
   position retraction into one function with no nested user calls, which the
   Solve IR does accept, so the synthesized barometer H can be evaluated by the
   rumoca binary itself. It is mathematically identical to
   EskfJacobianProof.retract_pos_z at delta = 0."

  function retract_pos_z_flat
    "World-up (ENU z) position after right-injecting a 15-tangent.
     Rrow is the third row of R(qBar), the constant world-up body axis."
    input Real pBar[3];
    input Real Rrow[3];
    input Real delta[15];
    output Real y;
  protected
    Real att[3];
    Real tsq;
    Real A;
    Real B;
    Real S[3, 3];
    Real S2[3, 3];
    Real J[3, 3];
    Real exppos[3];
  algorithm
    att := delta[7:9];
    tsq := att[1]^2 + att[2]^2 + att[3]^2;
    if tsq < 1e-2 then
      A := 0.5 - tsq/24.0;
      B := 1.0/6.0 - tsq/120.0;
    else
      A := (1.0 - cos(sqrt(tsq)))/tsq;
      B := (sqrt(tsq) - sin(sqrt(tsq)))/(tsq*sqrt(tsq));
    end if;
    S := {{0.0, -att[3], att[2]}, {att[3], 0.0, -att[1]}, {-att[2], att[1], 0.0}};
    S2 := S*S;
    J := identity(3) + A*S + B*S2;
    exppos := J*delta[1:3];
    y := pBar[3] + Rrow*exppos;
  end retract_pos_z_flat;

  model BaroFlat
    parameter Real pBar[3] = {12.0, -5.0, 30.0};
    parameter Real Rrow[3] = {0.489111368532, 0.153525790706, 0.858603459555};
    parameter Real delta[15] = zeros(15);
    Real Hsynth[1, 15] = jacobian(retract_pos_z_flat(pBar, Rrow, delta), delta);
    Real Hhand[1, 15] = cat(2, {Rrow}, zeros(1, 12));
    Real maxDiffHand = max(abs(Hsynth - Hhand));
    Real sizeH = max(abs(Hsynth));
    Real clock(start = 0, fixed = true);
  equation
    der(clock) = 0;
  end BaroFlat;
end EskfJacobianProofFlat;
