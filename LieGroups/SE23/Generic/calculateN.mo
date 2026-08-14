within LieGroups.SE23.Generic;
function calculateN "Closed-form mixed-exponential translational term"
  input Real tangent[9];
  input Real B[2, 2];
  output Real N[3, 2];
protected
  Real omega[3];
  Real Omega[3, 3];
  Real OmegaSquared[3, 3];
  Real A[3, 2];
  Real thetaSquared;
  Real theta;
  Real C1;
  Real C2;
  Real C3;
  Real I2[2, 2];
  // The closed coefficients are cancellation-prone below 0.1 rad in
  // single-precision generated code; the retained series is accurate there.
  constant Real epsilon = 1.0e-2;
algorithm
  omega := tangent[7:9];
  Omega := LieGroups.SO3.Quat.wedge(omega);
  OmegaSquared := Omega * Omega;
  A := [tangent[4:6], tangent[1:3]];
  I2 := [1.0, 0.0; 0.0, 1.0];
  thetaSquared := omega * omega;
  if thetaSquared < epsilon then
    C1 := 0.5 - thetaSquared / 24.0;
    C2 := 1.0 / 6.0 - thetaSquared / 120.0;
    C3 := 1.0 / 24.0 - thetaSquared / 720.0;
  else
    theta := sqrt(thetaSquared);
    C1 := (1.0 - cos(theta)) / thetaSquared;
    C2 := (theta - sin(theta)) / (thetaSquared * theta);
    C3 := (0.5 * thetaSquared + cos(theta) - 1.0)
      / (thetaSquared * thetaSquared);
  end if;
  N := A + 0.5 * A * B
    + Omega * A * (C1 * I2 + C2 * B)
    + OmegaSquared * A * (C2 * I2 + C3 * B);
end calculateN;
