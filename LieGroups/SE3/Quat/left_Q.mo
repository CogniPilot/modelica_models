within LieGroups.SE3.Quat;
function left_Q "Barfoot Q-matrix: off-diagonal block of the SE(3) left Jacobian"
  input Real rho[3] "Translational algebra part";
  input Real omega[3] "Rotation algebra part";
  output Real Q[3,3] "3x3 Q matrix";
protected
  Real theta_sq;
  Real theta;
  Real C1 "(t - sin t)/t^3";
  Real C2 "(t^2 + 2 cos t - 2)/(2 t^4)";
  Real C3 "(t cos t + 2 t - 3 sin t)/(2 t^5)";
  Real C4 "(t^2 + t sin t + 4 cos t - 4)/(2 t^6)";
  Real C5 "(2 - 2 cos t - t sin t)/(2 t^4)";
  Real V[3,3]  "[rho]x";
  Real O[3,3]  "[omega]x";
  Real O2[3,3], OV[3,3], VO[3,3], O2V[3,3], VO2[3,3];
  Real OVO2[3,3], O2VO[3,3], O2VO2[3,3], OVO[3,3];
  Real s, ct, st;
  constant Real eps = 1e-8;
algorithm
  theta_sq := omega[1]^2 + omega[2]^2 + omega[3]^2;
  s := theta_sq;
  if theta_sq < eps then
    // Taylor series in s = theta^2 (near identity)
    C1 := 1.0/6.0    - s/120.0    + s*s/5040.0;
    C2 := 1.0/24.0   - s/720.0    + s*s/40320.0;
    C3 := 1.0/120.0  - s/2520.0   + s*s/120960.0;
    C4 := 1.0/720.0  - s/20160.0  + s*s/1209600.0;
    C5 := 1.0/24.0   - s/360.0    + s*s/13440.0;
  else
    // NaN-safe: max(theta_sq, eps) is exact when this branch is taken (theta_sq >= eps)
    // and keeps the closed form finite under both-branch/AD evaluation at theta_sq = 0.
    s := max(theta_sq, eps);
    theta := sqrt(s);
    ct := cos(theta);
    st := sin(theta);
    C1 := (theta - st) / (s*theta);
    C2 := (s + 2.0*ct - 2.0) / (2.0*s^2);
    C3 := (theta*ct + 2.0*theta - 3.0*st) / (2.0*s^2*theta);
    C4 := (s + theta*st + 4.0*ct - 4.0) / (2.0*s^3);
    C5 := (2.0 - 2.0*ct - theta*st) / (2.0*s^2);
  end if;

  V := LieGroups.SO3.Quat.wedge(rho);
  O := LieGroups.SO3.Quat.wedge(omega);
  for i in 1:3 loop
    for j in 1:3 loop
      O2[i,j] := O[i,1]*O[1,j] + O[i,2]*O[2,j] + O[i,3]*O[3,j];
      OV[i,j] := O[i,1]*V[1,j] + O[i,2]*V[2,j] + O[i,3]*V[3,j];
      VO[i,j] := V[i,1]*O[1,j] + V[i,2]*O[2,j] + V[i,3]*O[3,j];
    end for;
  end for;

  for i in 1:3 loop
    for j in 1:3 loop
      O2V[i,j] := O2[i,1]*V[1,j] + O2[i,2]*V[2,j] + O2[i,3]*V[3,j];
      VO2[i,j] := V[i,1]*O2[1,j] + V[i,2]*O2[2,j] + V[i,3]*O2[3,j];
      OVO[i,j] := OV[i,1]*O[1,j] + OV[i,2]*O[2,j] + OV[i,3]*O[3,j];
      OVO2[i,j] := OV[i,1]*O2[1,j] + OV[i,2]*O2[2,j] + OV[i,3]*O2[3,j];
      O2VO[i,j] := O2V[i,1]*O[1,j] + O2V[i,2]*O[2,j] + O2V[i,3]*O[3,j];
      O2VO2[i,j] := O2V[i,1]*O2[1,j] + O2V[i,2]*O2[2,j] + O2V[i,3]*O2[3,j];
    end for;
  end for;

  for i in 1:3 loop
    for j in 1:3 loop
      Q[i,j] := 0.5*V[i,j]
        + C1*(OV[i,j] + VO[i,j])
        + C2*(O2V[i,j] + VO2[i,j])
        + C3*(OVO2[i,j] + O2VO[i,j])
        + C4*O2VO2[i,j]
        + C5*OVO[i,j];
    end for;
  end for;

  annotation(Documentation(info="<html>
    <p>SE(3) left-Jacobian Q block (Barfoot). The SE(3) left Jacobian is
    [[J_l(omega), Q],[0, J_l(omega)]]. Used to assemble the SE_2(3) left Jacobian.</p>
    <p>Q = (1/2)[rho]x + C1([w][rho]+[rho][w]) + C2([w]^2[rho]+[rho][w]^2)
       + C3([w][rho][w]^2+[w]^2[rho][w]) + C4([w]^2[rho][w]^2) + C5([w][rho][w])</p>
  </html>"));
end left_Q;
