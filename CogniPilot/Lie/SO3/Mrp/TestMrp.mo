within CogniPilot.Lie.SO3.Mrp;
encapsulated model TestMrp
  import CogniPilot.Lie.SO3;

  import CogniPilot.Lie.SO3.Mrp;

  SO3.Mrp.Group G;

  SO3.Algebra.Element g1;

  Mrp.Group.Kinematics kin(
    a0=Mrp.Element(
      r={0,0,0}),
    omega={1,2,3});

  Mrp.Element G1;

equation
  G1=kin.a;

  assert(
    G1 == G1,
    "equal failed");

  assert(
    G1*G.one() == G1,
    "multiplication by one failed");

  assert(
    G1*G1.inverse() == G.one(),
    "inverse test failed");

  g1=G1.log();

//assert(g1.exp(G) == G1, "log exp test failed");
end TestMrp;
