within SLAM.Localization;
function RGBDUncertaintyProper
  input Real R[3,3];
  output Boolean valid;
protected
  Real gram[3,3]; Real determinant;
algorithm
  valid := true;
  for a in 1:3 loop
    for b in 1:3 loop
      valid := valid and abs(R[a,b]) <= 1.000001;
    end for;
  end for;
  if valid then
    gram := transpose(R)*R;
    for a in 1:3 loop
      for b in 1:3 loop
        valid := valid and abs(gram[a,b]-(if a == b then 1.0 else 0.0)) <= 1e-6;
      end for;
    end for;
    determinant := R[1,1]*(R[2,2]*R[3,3]-R[2,3]*R[3,2])
      -R[1,2]*(R[2,1]*R[3,3]-R[2,3]*R[3,1])
      +R[1,3]*(R[2,1]*R[3,2]-R[2,2]*R[3,1]);
    valid := valid and abs(determinant-1.0) <= 1e-6;
  end if;
end RGBDUncertaintyProper;
