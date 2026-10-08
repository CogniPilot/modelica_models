within Vision.Registration;
function RegistrationPairResidual
  import RegistrationWhitenedResidual3 = Vision.Registration.RegistrationWhitenedResidual3;

  input Real source[3]; input Real target[3];
  input Real rotation[3,3]; input Real translation[3];
  input Boolean useCovariance;
  input Real sourceCovariance[3,3]; input Real targetCovariance[3,3];
  input Real minimumPivot;
  output Boolean valid; output Real squaredResidual;
protected
  Real residual[3]; Real covariance[3,3];
algorithm
  residual := rotation*source+translation-target;
  if useCovariance then
    covariance := rotation*sourceCovariance*transpose(rotation)+targetCovariance;
    (valid,squaredResidual) := RegistrationWhitenedResidual3(residual,covariance,minimumPivot);
  else
    squaredResidual := residual*residual;
    valid := squaredResidual >= 0.0 and squaredResidual < 1e100;
  end if;
end RegistrationPairResidual;
