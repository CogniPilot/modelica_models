within Vision.Registration;
// Covariance-weighted refinement in the optical translation/left-angle chart.
// Depth uncertainty is anisotropic: equal-weight Horn fits are only seeds.
package RegistrationWeighted
  function WeightedSkew
    input Real v[3]; output Real result[3,3];
  algorithm
    result := [0,-v[3],v[2];v[3],0,-v[1];-v[2],v[1],0];
  end WeightedSkew;

  function WeightedExp
    input Real v[3]; output Real result[3,3];
  protected
    Real square; Real angle; Real a; Real b; Real K[3,3];
  algorithm
    square := v*v; angle := sqrt(square);
    a := if square < 1e-8 then 1-square/6+square^2/120 else sin(angle)/angle;
    b := if square < 1e-8 then 0.5-square/24+square^2/720 else (1-cos(angle))/square;
    K := WeightedSkew(v); result := identity(3)+a*K+b*K*K;
  end WeightedExp;

// SPD solve with dimensionless diagonal equilibration, no damping/floor added
// to the matrix. A small or nonfinite pivot is a refusal, not invented noise.
function WeightedInverse6
  input Real A[6,6];
  input Real minimumPivot;
  output Real inverseA[6,6];
  output Boolean valid;
  output Real minimumScaledPivot;
protected
  Real scale[6]; Real L[21]; Real v; Real z[6]; Real x[6];
algorithm
  inverseA := zeros(6,6); L := zeros(21); scale := ones(6);
  minimumScaledPivot := 1.0; valid := true;
  for i in 1:6 loop
    valid := valid and A[i,i] > 0.0 and A[i,i] <= 1e100;
    scale[i] := sqrt(if A[i,i] > 0.0 and A[i,i] <= 1e100 then A[i,i] else 1.0);
    for j in 1:6 loop
      valid := valid and abs(A[i,j]) <= 1e100
        and abs(A[i,j]-A[j,i]) <= 1e-10*max(1.0,abs(A[i,j]));
    end for;
  end for;
  for i in 1:6 loop
    for j in 1:6 loop
      v := if valid then A[i,j]/scale[i]/scale[j] else 0.0;
      for k in 1:6 loop
        v := v-(if k < j and k <= i then L[LinearAlgebra.LowerTriangular.index(i, k)]*L[LinearAlgebra.LowerTriangular.index(j, k)] else 0.0);
      end for;
      if j <= i then
        if j == i then
          minimumScaledPivot := min(minimumScaledPivot,v);
          valid := valid and v > minimumPivot and v <= 1e100;
          L[LinearAlgebra.LowerTriangular.index(i, j)] := sqrt(if valid then v else 1.0);
        else
          L[LinearAlgebra.LowerTriangular.index(i, j)] := if valid then v/L[LinearAlgebra.LowerTriangular.index(j, j)] else 0.0;
        end if;
      end if;
    end for;
  end for;
  for column in 1:6 loop
    z := zeros(6); x := zeros(6);
    for i in 1:6 loop
      v := if i == column then 1.0/scale[column] else 0.0;
      for k in 1:6 loop
        v := v-(if k < i then L[LinearAlgebra.LowerTriangular.index(i, k)]*z[k] else 0.0);
      end for;
      z[i] := if valid then v/L[LinearAlgebra.LowerTriangular.index(i, i)] else 0.0;
    end for;
    for index in 1:6 loop
      v := z[7-index];
      for k in 1:6 loop
        v := v-(if k > 7-index then L[LinearAlgebra.LowerTriangular.index(k, 7-index)]*x[k] else 0.0);
      end for;
      x[7-index] := if valid then v/L[LinearAlgebra.LowerTriangular.index(7-index, 7-index)] else 0.0;
    end for;
    for i in 1:6 loop
      inverseA[i,column] := if valid then x[i]/scale[i] else 0.0;
    end for;
  end for;
  inverseA := if valid then inverseA else zeros(6,6);
end WeightedInverse6;

  function WeightedWhiten
    input Real covariance[3,3]; input Real residual[3]; input Real jacobian[3,6];
    input Real minimumPivot;
    output Boolean valid; output Real wr[3]; output Real wj[3,6];
  protected
    Real scale[3]; Real L[6]; Real v;
  algorithm
    valid := minimumPivot > 0 and minimumPivot < 1;
    scale := ones(3); L := zeros(6); wr := zeros(3); wj := zeros(3,6);
    for row in 1:3 loop
      valid := valid and covariance[row,row] > 0 and covariance[row,row] < 1e100;
      if valid then scale[row] := sqrt(covariance[row,row]); end if;
    end for;
    for row in 1:3 loop
      for column in 1:row loop
        v := covariance[row,column]/scale[row]/scale[column];
        valid := valid and abs(v) < 1e100
          and abs(covariance[row,column]-covariance[column,row]) <= 1e-10*max(1.0,abs(covariance[row,column]));
        for k in 1:column-1 loop v := v-L[LinearAlgebra.LowerTriangular.index(row, k)]*L[LinearAlgebra.LowerTriangular.index(column, k)]; end for;
        if row == column then
          valid := valid and v > minimumPivot and v < 1e100;
          if valid then L[LinearAlgebra.LowerTriangular.index(row, column)] := sqrt(v); end if;
        elseif valid then L[LinearAlgebra.LowerTriangular.index(row, column)] := v/L[LinearAlgebra.LowerTriangular.index(column, column)]; end if;
      end for;
      v := if valid then residual[row]/scale[row] else 0.0;
      for k in 1:row-1 loop v := v-L[LinearAlgebra.LowerTriangular.index(row, k)]*wr[k]; end for;
      if valid then wr[row] := v/L[LinearAlgebra.LowerTriangular.index(row, row)]; end if;
      for column in 1:6 loop
        v := if valid then jacobian[row,column]/scale[row] else 0.0;
        for k in 1:row-1 loop v := v-L[LinearAlgebra.LowerTriangular.index(row, k)]*wj[k,column]; end for;
        if valid then wj[row,column] := v/L[LinearAlgebra.LowerTriangular.index(row, row)]; end if;
      end for;
    end for;
  end WeightedWhiten;

  function WeightedNormal
    input Real sourcePoint[:,3]; input Real targetPoint[size(sourcePoint,1),3];
    input Real enabled[size(sourcePoint,1)];
    input Real sourceCovariance[size(sourcePoint,1),3,3]; input Real targetCovariance[size(sourcePoint,1),3,3];
    input Real rotation[3,3]; input Real translation[3]; input Real minimumPivot;
    output Boolean valid; output Real cost; output Real H[6,6]; output Real gradient[6]; output Real rms;
  protected
    Boolean pointValid; Real q[3]; Real r[3]; Real J[3,6]; Real wr[3]; Real wj[3,6];
    Real covariance[3,3]; Real informationPacked[21]; Real squared; Integer count;
  algorithm
    valid := true; cost := 0; informationPacked := zeros(21); gradient := zeros(6); squared := 0; count := 0;
    for point in 1:size(sourcePoint,1) loop
      if enabled[point] == 1.0 then
        q := rotation*sourcePoint[point,:]; r := q+translation-targetPoint[point,:];
        J := cat(2,identity(3),-WeightedSkew(q));
        covariance := targetCovariance[point,:,:]+LinearAlgebra.transformCovariance(rotation,sourceCovariance[point,:,:]);
        (pointValid,wr,wj) := WeightedWhiten(covariance,r,J,minimumPivot);
        valid := valid and pointValid;
        informationPacked := informationPacked+LinearAlgebra.SymmetricMatrix.gram(wj); gradient := gradient+transpose(wj)*wr;
        cost := cost+wr*wr; squared := squared+r*r; count := count+1;
      end if;
    end for;
    H := LinearAlgebra.SymmetricMatrix.unpack(6,informationPacked);
    rms := if count > 0 then sqrt(squared/count) else 0;
    valid := valid and count >= 3 and cost >= 0 and cost < 1e100;
  end WeightedNormal;

  function WeightedRefine
    input Real sourcePoint[:,3]; input Real targetPoint[size(sourcePoint,1),3];
    input Real enabled[size(sourcePoint,1)];
    input Real sourceCovariance[size(sourcePoint,1),3,3]; input Real targetCovariance[size(sourcePoint,1),3,3];
    input Real initialRotation[3,3]; input Real initialTranslation[3]; input Real minimumPivot = 1e-10;
    output Boolean valid; output Real rotation[3,3]; output Real translation[3]; output Real rms;
  protected
    Real H[6,6]; Real inverseH[6,6]; Real gradient[6]; Real step[6]; Real cost; Real pivot;
    Real trialH[6,6]; Real trialGradient[6]; Real trialCost; Real trialRms; Real scale;
    Real trialRotation[3,3]; Real trialTranslation[3]; Boolean normalValid; Boolean inverseValid;
    Boolean trialValid; Boolean accepted; Boolean running;
  algorithm
    rotation := initialRotation; translation := initialTranslation;
    (normalValid,cost,H,gradient,rms) := WeightedNormal(sourcePoint,targetPoint,enabled,sourceCovariance,targetCovariance,rotation,translation,minimumPivot);
    (inverseH,inverseValid,pivot) := WeightedInverse6(H,minimumPivot);
    valid := normalValid and inverseValid; running := valid;
    step := zeros(6); scale := 1.0; accepted := false;
    for iteration in 1:8 loop
      if running then
        step := -inverseH*gradient;
        scale := min(1.0,min(0.5/max(1e-12,sqrt(step[1:3]*step[1:3])),0.1/max(1e-12,sqrt(step[4:6]*step[4:6]))));
        accepted := false;
      end if;
      // Keep the bounded loop outside the conditional so the pinned Rumoca
      // release can lower this function; guarded updates preserve early stop.
      for backtrack in 1:8 loop
          if running and not accepted then
            trialRotation := WeightedExp(scale*step[4:6])*rotation; trialTranslation := translation+scale*step[1:3];
            (trialValid,trialCost,trialH,trialGradient,trialRms) := WeightedNormal(sourcePoint,targetPoint,enabled,sourceCovariance,targetCovariance,trialRotation,trialTranslation,minimumPivot);
            if trialValid and trialCost <= cost then
              rotation := trialRotation; translation := trialTranslation; cost := trialCost;
              H := trialH; gradient := trialGradient; rms := trialRms; accepted := true;
            else scale := scale*0.5; end if;
          end if;
      end for;
      if running then
        running := accepted and scale*sqrt(step*step) > 1e-7;
        (inverseH,inverseValid,pivot) := WeightedInverse6(H,minimumPivot);
        valid := valid and inverseValid; running := running and valid;
      end if;
    end for;
  end WeightedRefine;
end RegistrationWeighted;
