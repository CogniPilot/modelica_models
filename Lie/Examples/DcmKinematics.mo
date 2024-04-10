within Lie.Examples;

model DcmKinematics
  import Lie.SO3.Dcm;
  
  constant input Dcm C0;

  input Real[3] omega;

  parameter Lie.TangentSpace tangentSpace;

  output Dcm C;

  output Real orthoNormError;

  parameter Real tol=1e-4;

initial equation
  C.r=C0.r;

equation
  der(
    C.r)=Lie.SO3.Dcm.deriv(
    C,
    omega,
    tangentSpace);

  orthoNormError=Lie.SO3.Dcm.orthoNormError(
    C);

  when(orthoNormError > tol) then
    reinit(
      C.r,
      Lie.SO3.Dcm.orthoNormalize(
        pre(
          C.r)));

  end when;

end DcmKinematics;