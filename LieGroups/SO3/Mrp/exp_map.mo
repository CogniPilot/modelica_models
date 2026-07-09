within LieGroups.SO3.Mrp;
function exp_map "Exponential map: so(3) vector -> MRP"
  input Real v[3] "Rotation vector (axis * angle)";
  output Real r[3] "MRP parameters";
protected
  Real theta_sq, theta;
  Real A "tan(theta/4) / theta";
  Real n_sq;
  Real r_shadow[3];
  Real r_next[3];
  constant Real eps = 1e-8;
algorithm
  theta_sq := v[1]^2 + v[2]^2 + v[3]^2;

  if theta_sq < eps then
    // Taylor: tan(t/4)/t ~ 1/4 + t^2/192
    A := 0.25 + theta_sq / 192.0;
  else
    theta := sqrt(theta_sq);
    A := tan(theta / 4.0) / theta;
  end if;

  for i in 1:3 loop
    r[i] := A * v[i];
  end for;

  // Shadow switch if ||r|| > 1.
  n_sq := r[1]^2 + r[2]^2 + r[3]^2;
  r_shadow := LieGroups.SO3.Mrp.shadow(r);
  if n_sq > 1.0 then
    for i in 1:3 loop
      r_next[i] := r_shadow[i];
    end for;
  else
    for i in 1:3 loop
      r_next[i] := r[i];
    end for;
  end if;
  for i in 1:3 loop
    r[i] := r_next[i];
  end for;
end exp_map;
