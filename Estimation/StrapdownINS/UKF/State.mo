within Estimation.StrapdownINS.UKF;

record State "Nominal strapdown state and local 15-state covariance"
  extends Estimation.StrapdownINS.ESKF.NominalState;
  Estimation.StrapdownINS.UKF.Covariance covariance;
end State;
