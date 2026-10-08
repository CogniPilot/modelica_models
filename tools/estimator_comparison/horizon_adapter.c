#define _POSIX_C_SOURCE 200809L
#include "Vehicles_Rdd2_NavigationEstimator.h"
#include "replay_timing.h"
#include "transport.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static NavigationEstimatorState estimator;
static void copy3(float out[3], const float in[3]) {
  memcpy(out, in, 3 * sizeof(float));
}
#include "horizon.h"

#define CHECK_FIXED(field)                                                     \
  if (memcmp(&aiding.field, &initial.field, sizeof(aiding.field))) {           \
    fprintf(stderr, "Arrival overwrote fixed aiding configuration: %s\n",      \
            #field);                                                           \
    return 1;                                                                  \
  }

int main(void) {
  NavigationEstimator_startup(&estimator);
  for (unsigned axis = 0; axis < 3; ++axis) {
    estimator.gps_positionCovarianceWorld_m2[axis][axis] = .04f;
    estimator.velocityCovarianceWorld_m2_s2[axis][axis] = .0025f;
    estimator.covarianceBody_T2[axis][axis] = 9e-14f;
    estimator.integratedGyroscopeCovariance_rad2[axis][axis] = 1e-10f;
  }
  for (unsigned axis = 0; axis < 2; ++axis)
    estimator.integratedLineOfSightCovariance_rad2[axis][axis] = 1e-8f;
  estimator.variance_m2 = .09f;
  estimator.opticalFlow_integrationTime_s = .01f;
  estimator.groundDistanceVariance_m2 = .01f;
  estimator.quality = 255;
  const double delays[4] = {.11, .03, .02, .02};
  horizon_startup(true, delays, .01);
  static AidingBufferState initial;
  initial = aiding;
  memset(&estimator, 0, sizeof(estimator));
  estimator.gps_valid = estimator.gps_fresh = true;
  estimator.gps_timestamp_s = 1.25f;
  estimator.gps_positionWorldEnu_m[0] = 42;
  horizon_receive(false);
  CHECK_FIXED(gps_positionCovarianceWorld_m2);
  CHECK_FIXED(gps_velocityCovarianceWorld_m2_s2);
  CHECK_FIXED(magnetometer_covarianceBody_T2);
  CHECK_FIXED(barometer_variance_m2);
  CHECK_FIXED(opticalFlow_integratedLineOfSightCovariance_rad2);
  CHECK_FIXED(opticalFlow_integratedGyroscopeCovariance_rad2);
  CHECK_FIXED(opticalFlow_integrationTime_s);
  CHECK_FIXED(opticalFlow_groundDistanceVariance_m2);
  CHECK_FIXED(opticalFlow_quality);
  if (!aiding.gps_valid || !aiding.gps_fresh ||
      aiding.gps_timestamp_s != 1.25f || aiding.gps_positionWorldEnu_m[0] != 42)
    return 1;
  estimator.opticalFlow_integrationTime_s = .1f;
  estimator.integratedLineOfSightCovariance_rad2[0][0] = 8.55e-7f;
  estimator.integratedGyroscopeCovariance_rad2[0][0] = 2.8e-10f;
  horizon_receive(true);
  if (aiding.opticalFlow_integrationTime_s != .1f ||
      memcmp(aiding.opticalFlow_integratedLineOfSightCovariance_rad2,
             estimator.integratedLineOfSightCovariance_rad2,
             sizeof(estimator.integratedLineOfSightCovariance_rad2)) ||
      memcmp(aiding.opticalFlow_integratedGyroscopeCovariance_rad2,
             estimator.integratedGyroscopeCovariance_rad2,
             sizeof(estimator.integratedGyroscopeCovariance_rad2)))
    return 1;
  CHECK_FIXED(gps_positionCovarianceWorld_m2);
  puts("Horizon adapter transferred explicit exposure duration and covariance");
  memset(&estimator, 0, sizeof(estimator));
  NavigationEstimator_startup(&estimator);
  estimator.pseudoPositionVariance_m2 = 0;
  estimator.zeroVelocityVariance_m2_s2 = 0;
  horizon_startup(true, delays, .01);
  const float gyro[3] = {0}, accel[3] = {0, 0, 9.81f};
  bool delayed_rest_observed = false, released_rest_observed = false;
  for (unsigned tick = 0; tick < 800; ++tick) {
    const int32_t count = estimator.status_acceptedCorrectionCount;
    if (!horizon_tick(gyro, accel, tick % 8 == 0, .5))
      return 1;
    if (!predictor.valid || !estimator.status_predictionAccepted)
      continue;
    const bool at_rest = estimator.imu_timestamp_s < .5;
    if (estimator.vehicleAtRest != at_rest ||
        estimator.status_zeroVelocityCorrectionAccepted != at_rest ||
        estimator.status_acceptedCorrectionCount != count + at_rest) {
      fprintf(stderr,
              "Stationary constraint used arrival time or lost a correction\n");
      return 1;
    }
    delayed_rest_observed |= tick > 400 && at_rest;
    released_rest_observed |= !at_rest;
  }
  if (!delayed_rest_observed || !released_rest_observed)
    return 1;
  puts("Horizon stationary constraint followed fusion time and notified "
       "rebases");
  puts("Horizon adapter preserved aiding configuration and original payload");
  return 0;
}
