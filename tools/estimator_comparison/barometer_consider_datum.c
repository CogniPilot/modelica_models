/* SPDX-License-Identifier: Apache-2.0 */
#include "Vehicles_Rdd2_NavigationEstimator.h"
#include <math.h>
#include <stdio.h>

static NavigationEstimatorState estimator;

int main(void) {
  NavigationEstimator_startup(&estimator);
  estimator.useBarometerBiasConsider = true;
  estimator.useDeclaredRestBarometerCalibration = true;
  estimator.barometerBiasCalibrationSamples = 2;
  estimator.barometerBiasProcessNoise_m2_s = .1f;
  estimator.initialAlignmentWindow_s = 0;
  estimator.initialAlignmentTimeout_s = 0;
  estimator.pseudoPositionVariance_m2 = 0;
  estimator.zeroVelocityVariance_m2_s2 = 0;
  estimator.imu_valid = estimator.imu_fresh = true;
  estimator.imu_integrationTime_s = .01f;
  estimator.specificForceBodyFlu_m_s2[2] = 9.81f;
  estimator.deltaVelocityBodyFlu_m_s[2] = .0981f;
  estimator.deltaPositionBodyFlu_m[2] = .0004905f;
  estimator.deltaQuaternionBodyFlu[0] = 1;
  estimator.barometer_valid = estimator.barometer_fresh = true;
  estimator.altitudeWorldEnu_m = .2f;
  estimator.variance_m2 = .01f;
  double variance = estimator.initialBarometerBiasVariance_m2;
  double mean = estimator.initialBarometerBias_m;
  for (unsigned sample = 0; sample < 5; ++sample) {
    estimator.imu_timestamp_s = .1f + sample * .01f;
    estimator.barometer_timestamp_s = estimator.imu_timestamp_s - .02f;
    estimator.vehicleAtRest = true;
    const double age = (double)estimator.imu_timestamp_s - estimator.barometer_timestamp_s;
    const double diffusion = estimator.barometerBiasProcessNoise_m2_s;
    const double predicted = variance + diffusion * estimator.samplePeriod;
    const double observed = predicted - diffusion * age;
    const double gain = observed / (observed + estimator.variance_m2);
    mean += gain * (estimator.altitudeWorldEnu_m - mean);
    variance = (1 - gain) * observed + diffusion * age;
    NavigationEstimator_dostep(&estimator);
    if (estimator.rumoca_galec_error_signal_status ||
        fabs(estimator.barometerBias_m - mean) > 2e-6 ||
        fabs(estimator.barometerBiasVariance_m2 - variance) > 2e-6 ||
        estimator.status_barometerCorrectionAccepted) {
      fprintf(stderr, "Delayed datum scalar oracle failed at packet %u\n", sample);
      return 1;
    }
    for (unsigned axis = 0; axis < 15; ++axis)
      if (estimator.barometerBiasCrossCovariance[axis] != 0) {
        fputs("Withheld pressure calibration acquired navigation correlation\n", stderr);
        return 1;
      }
  }
  estimator.vehicleAtRest = false;
  estimator.imu_timestamp_s += .01f;
  estimator.barometer_timestamp_s += .01f;
  NavigationEstimator_dostep(&estimator);
  if (estimator.rumoca_galec_error_signal_status ||
      !estimator.status_barometerCorrectionAccepted ||
      fabs(estimator.barometerBiasCrossCovariance[2]) < 1e-6) {
    fputs("Released pressure fusion lost its datum correlation\n", stderr);
    return 1;
  }
  puts("Delayed scalar pressure-datum oracle, withholding and fusion correlation passed");
  return 0;
}
