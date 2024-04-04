within CogniPilot.Lie;
model Test
  Real omega[3]={1,2,3};
  Real R[3,3];
  CogniPilot.Lie.SO3.Mrp.Kinematics mrp(
    a0=SO3.Mrp.Element(
      r={0,0,0}),
    omega=omega);
equation
  R=mrp.a.to_matrix();
end Test;
