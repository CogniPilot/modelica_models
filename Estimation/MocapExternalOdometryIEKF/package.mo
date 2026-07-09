within Estimation;
package MocapExternalOdometryIEKF
  constant Integer StateLength = 157
    "Flat state length: attitude, velocity, position, angular velocity, covariance";
  constant Integer TangentLength = 12
    "Tangent covariance dimension";
end MocapExternalOdometryIEKF;
