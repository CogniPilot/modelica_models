within Estimation.StrapdownINS.ESKF;

record State "Nominal state and full local-error tangent covariance"
  extends Estimation.StrapdownINS.ESKF.NominalState;
  Estimation.StrapdownINS.ESKF.Covariance covariance;
end State;
