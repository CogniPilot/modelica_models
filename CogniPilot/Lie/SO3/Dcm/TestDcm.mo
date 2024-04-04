within CogniPilot.Lie.SO3.Dcm;
encapsulated model TestDcm
  import CogniPilot.Lie.SO3;

  import CogniPilot.Lie.SO3.Dcm;

  SO3.Dcm.Group G;

  Dcm.Group.Kinematics kin(
    a0=G.one(),
    omega={1,2,3});

  Dcm.Element G1;

equation
  G1=kin.a;

  assert(
    G1 == G1,
    "equal failed");

  assert(
    G1*G.one() == G1,
    "multiplication by one failed");

  assert(
    G.equal(
      G1*G1.inverse(),
      G.one(),
      1e-4),
    "inverse test failed");

end TestDcm;
