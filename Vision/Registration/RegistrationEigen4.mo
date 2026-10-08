within Vision.Registration;
// Horn absolute orientation for matched 3D point pairs. No pose truth enters
// this component. Correspondence production and temporal pose composition are
// separate frontend/backend responsibilities.
function RegistrationEigen4
  input Real matrix[4,4];
  output Real values[4];
  output Real dominant[4];
  output Real gap;
  output Real offDiagonal;
protected
  Real work[4,4];
  Real vectors[4,4];
  Real scale;
  Real tau; Real tangent; Real cosine; Real sine;
  Real first; Real second; Real entry; Real active;
  Real largest; Real runnerUp;
  Integer index;
algorithm
  scale := 0.0;
  for i in 1:4 loop
    for j in 1:4 loop
      scale := max(scale,abs(matrix[i,j]));
    end for;
  end for;
  work := matrix/(if scale > 0.0 then scale else 1.0);
  vectors := identity(4);
  for sweep in 1:24 loop
    for p in 1:3 loop
      for q in 1:4 loop
        entry := work[p,q];
        active := if q > p and abs(entry) > 1e-14 then 1.0 else 0.0;
        tau := (work[q,q]-work[p,p])/(if active > 0.0 then 2.0*entry else 1.0);
        tangent := if active <= 0.0 then 0.0 else if tau >= 0.0 then
          1.0/(tau+sqrt(1.0+tau*tau)) else -1.0/(-tau+sqrt(1.0+tau*tau));
        cosine := 1.0/sqrt(1.0+tangent*tangent);
        sine := tangent*cosine;
        work[p,p] := work[p,p]-tangent*entry;
        work[q,q] := work[q,q]+tangent*entry;
        work[p,q] := if active > 0.0 then 0.0 else entry;
        work[q,p] := work[p,q];
        for k in 1:4 loop
          first := work[k,p]; second := work[k,q];
          work[k,p] := if k <> p and k <> q then cosine*first-sine*second else first;
          work[p,k] := if k <> p and k <> q then work[k,p] else work[p,k];
          work[k,q] := if k <> p and k <> q then sine*first+cosine*second else second;
          work[q,k] := if k <> p and k <> q then work[k,q] else work[q,k];
          first := vectors[k,p]; second := vectors[k,q];
          vectors[k,p] := cosine*first-sine*second;
          vectors[k,q] := sine*first+cosine*second;
        end for;
      end for;
    end for;
  end for;
  largest := work[1,1]; runnerUp := -1e100; index := 1;
  for i in 2:4 loop
    if work[i,i] > largest then
      runnerUp := largest; largest := work[i,i]; index := i;
    else
      runnerUp := max(runnerUp,work[i,i]);
    end if;
  end for;
  offDiagonal := 0.0;
  for i in 1:4 loop
    values[i] := work[i,i]*scale;
    dominant[i] := vectors[i,index];
    for j in 1:4 loop
      offDiagonal := if i <> j then max(offDiagonal,abs(work[i,j])) else offDiagonal;
    end for;
  end for;
  gap := (largest-runnerUp)*scale;
end RegistrationEigen4;
