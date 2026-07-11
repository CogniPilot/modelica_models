within LinearAlgebra;
function josephUpdate "Dimension-generic Joseph-form covariance correction"
  input Real factor[:, size(factor, 1)] "I - K*H";
  input Real covariance[size(factor, 1), size(factor, 1)]
    "Predicted covariance";
  input Real gain[size(factor, 1), :] "Kalman gain";
  input Real measurementNoise[size(gain, 2), size(gain, 2)]
    "Measurement covariance";
  output Real covarianceNext[size(factor, 1), size(factor, 1)];
algorithm
  covarianceNext := factor * covariance * transpose(factor)
    + gain * measurementNoise * transpose(gain);
end josephUpdate;
