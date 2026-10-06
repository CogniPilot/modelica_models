within Estimation.StrapdownINS.ESKF;

function seedPositionVariances
  "Bound an aiding position seed's uncertainty in the local tangent frame"
  input Real covarianceWorld_m2[3, 3];
  input Real quaternionWorldBody[4];
  input Real configuredVariances_m2[3];
  output Real variances_m2[3];
protected
  Boolean covarianceFinite;
  Boolean covariancePositive;
  Real probe[3, 1];
  Real rotationWorldBody[3, 3];
  Real covarianceBody_m2[3, 3];
  Real rowBound;
algorithm
  variances_m2 := configuredVariances_m2;
  covarianceFinite := true;
  for i in 1:3 loop
    for j in 1:3 loop
      covarianceFinite := covarianceFinite
        and abs(covarianceWorld_m2[i, j]) < FiniteMagnitudeLimit;
    end for;
  end for;
  if covarianceFinite then
    (probe, covariancePositive) := LinearAlgebra.solveSPD(
      LinearAlgebra.symmetrize(covarianceWorld_m2), zeros(3, 1));
    if covariancePositive then
      rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
        LieGroups.SO3.Quat.normalize(quaternionWorldBody));
      covarianceBody_m2 := transpose(rotationWorldBody)
        * LinearAlgebra.symmetrize(covarianceWorld_m2) * rotationWorldBody;
      for i in 1:3 loop
        // The initializer stores diagonal tangent variances. Absolute row
        // sums give a diagonal majorant of the full rotated covariance,
        // retaining configured floors and bounding off-diagonal terms.
        rowBound := 0.0;
        for j in 1:3 loop
          rowBound := rowBound + abs(covarianceBody_m2[i, j]);
        end for;
        variances_m2[i] := max(configuredVariances_m2[i], rowBound);
      end for;
    end if;
  end if;
end seedPositionVariances;
