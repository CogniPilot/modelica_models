within Lie.Examples;
model TestSO3
  "Tests SO3"
  import Lie.SO3;

  constant Real omega[3]={1,2,3};

  SO3.Algebra a(
    r={1,2,3});

  Real Jl[3,3]=SO3.Algebra.leftJac(
    a);

  Real Jl_inv[3,3]=SO3.Algebra.leftJacInv(
    a);

  Real Jr[3,3]=SO3.Algebra.rightJac(
    a);

  Real Jr_inv[3,3]=Lie.SO3.Algebra.rightJacInv(
    a);

  SO3.Algebra g(
    r={1,2,3});

  SO3.Dcm G=SO3.Algebra.expDcm(
    g);

  SO3.Algebra g1=SO3.Dcm.log(
    G);

  SO3.Euler e1(
    angles={0,0,0},
    axisSeq={1,2,3});

  constant SO3.Dcm C0=SO3.Dcm.fromEuler(
    SO3.Euler(
      angles={0.1,0.2,0.3},
      axisSeq={3,2,1},
      rotType=SO3.Euler.RotationType.Body));

  SO3.Dcm.Kinematics euler_kin(
    C0=C0,
    omega=omega,
    tangentSpace=TangentSpace.Left);

end TestSO3;
