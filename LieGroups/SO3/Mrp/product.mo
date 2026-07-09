within LieGroups.SO3.Mrp;
function product "MRP composition"
  input Real a[3] "Left MRP";
  input Real b[3] "Right MRP";
  output Real r[3] "Result MRP";
protected
  Real na_sq, nb_sq, dot_ab, denom;
  Real cross_ba[3];
  Real n_sq;
  Real r_shadow[3];
  Real r_next[3];
algorithm
  na_sq := a[1]^2 + a[2]^2 + a[3]^2;
  nb_sq := b[1]^2 + b[2]^2 + b[3]^2;
  dot_ab := a[1]*b[1] + a[2]*b[2] + a[3]*b[3];

  // cross(b, a)
  cross_ba := {b[2]*a[3] - b[3]*a[2],
               b[3]*a[1] - b[1]*a[3],
               b[1]*a[2] - b[2]*a[1]};

  denom := 1.0 + na_sq*nb_sq - 2.0*dot_ab;
  if abs(denom) < 1e-10 then
    denom := 1e-10;
  end if;

  for i in 1:3 loop
    r[i] := ((1.0 - na_sq)*b[i] + (1.0 - nb_sq)*a[i] - 2.0*cross_ba[i]) / denom;
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
end product;
