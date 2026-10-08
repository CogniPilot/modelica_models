/* SPDX-License-Identifier: Apache-2.0 */
#include "Vehicles_Rdd2_NavigationEstimator.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static NavigationEstimatorState s;
#define CHECK(condition, message)                                              \
  do {                                                                         \
    if (!(condition)) {                                                        \
      fprintf(stderr, "tick %d: %s\n", tick, message);                         \
      return 1;                                                                \
    }                                                                          \
  } while (0)

static bool covariance_positive_definite(const float covariance[15][15]) {
  double factor[15][15] = {{0}};
  for (unsigned row = 0; row < 15; ++row) {
    for (unsigned column = 0; column <= row; ++column) {
      double residual = covariance[row][column];
      if (!isfinite(residual) ||
          fabs(residual - covariance[column][row]) > 1e-6)
        return false;
      for (unsigned inner = 0; inner < column; ++inner)
        residual -= factor[row][inner] * factor[column][inner];
      if (row == column) {
        if (residual <= 0)
          return false;
        factor[row][column] = sqrt(residual);
      } else {
        factor[row][column] = residual / factor[column][column];
      }
    }
  }
  return true;
}

static int check_aiding(bool vector_fusion, bool geometric_alignment,
                        bool stationary, float stationary_variance,
                        bool semi_direct_bias) {
  memset(&s, 0, sizeof(s));
  NavigationEstimator_startup(&s);
  s.useEquivariantMagnetometer = vector_fusion;
  s.useGeometricAlignment = geometric_alignment;
  s.useSemiDirectBias = semi_direct_bias;
  s.barometerBiasCalibrationSamples = 5;
  s.zeroVelocityVariance_m2_s2 = 0;
  s.stationaryVelocityVariance_m2_s2 = stationary_variance;
  const bool stationary_configured =
      isfinite(stationary_variance) && stationary_variance > 0;
  s.initialTerrainAltitudeWorldEnu_m = -1;
  s.opticalFlowGroundPlaneOffset_m = -1;
  s.groundDistance_m = 1;
  s.groundDistanceVariance_m2 = .0004f;
  s.opticalFlow_integrationTime_s = .01f;
  s.imu_integrationTime_s = .01f;
  s.imu_valid = s.imu_fresh = true;
  s.opticalFlow_valid = s.opticalFlow_fresh = true;
  s.magnetometer_valid = s.magnetometer_fresh = true;
  s.barometer_valid = s.barometer_fresh = true;
  s.gps_fresh = true;
  s.specificForceBodyFlu_m_s2[2] = 9.81f;
  s.deltaVelocityBodyFlu_m_s[2] = .0981f;
  s.deltaPositionBodyFlu_m[2] = .0004905f;
  s.deltaQuaternionBodyFlu[0] = 1;
  s.variance_m2 = .01f;
  for (int axis = 0; axis < 3; ++axis) {
    s.magneticFieldBodyFlu_T[axis] = s.localMagneticFieldWorldEnu_T[axis];
    s.covarianceBody_T2[axis][axis] = 1e-13f;
    s.gps_positionCovarianceWorld_m2[axis][axis] = .04f;
    s.velocityCovarianceWorld_m2_s2[axis][axis] = .0025f;
    s.integratedGyroscopeCovariance_rad2[axis][axis] = 1e-10f;
    s.deltaRotationGyroscopeBiasJacobian_s[axis][axis] = -.01f;
    s.deltaVelocityAccelerometerBiasJacobian_s[axis][axis] = -.01f;
    s.deltaPositionAccelerometerBiasJacobian_s2[axis][axis] = -.00005f;
  }
  for (int axis = 0; axis < 2; ++axis)
    s.integratedLineOfSightCovariance_rad2[axis][axis] = 1e-8f;
  for (int tick = 0; tick < 200; ++tick) {
    s.imu_timestamp_s = s.opticalFlow_timestamp_s = tick * .01f;
    const int bad_flow = tick >= 80 && tick < 90;
    const int auxiliary_new = tick % 2 == 0 || bad_flow;
    s.magnetometer_timestamp_s = s.barometer_timestamp_s =
        (bad_flow ? tick : 2 * (tick / 2)) * .01f;
    s.gps_valid = s.positionValid = s.velocityValid = tick < 100 || tick >= 150;
    s.gps_timestamp_s = (tick / 10) * .1f;
    s.gps_positionWorldEnu_m[0] = tick >= 60 && tick < 70 ? 1000 : 0;
    s.quality = bad_flow ? 0 : 1;
    s.vehicleAtRest = stationary && tick < 120;
    const int count = s.status_acceptedCorrectionCount;
    const bool was_initialized = s.status_initialized;
    NavigationEstimator_dostep(&s);
    if (!isfinite(stationary_variance) && s.rumoca_galec_error_signal_status) {
      CHECK(!s.status_zeroVelocityCorrectionAccepted,
            "nonfinite stationary variance accepted a correction");
      puts("Generated C refused nonfinite stationary variance");
      return 0;
    }
    CHECK(!s.rumoca_galec_error_signal_status, "generated C error signal");
    if (geometric_alignment && !was_initialized && s.status_initialized) {
      CHECK(s.errorCovariance[6][6] < .1f && s.errorCovariance[7][7] < .1f,
            "quiet alignment did not condition initial tilt uncertainty");
      CHECK(s.magnetometerTimestampConsumed_s == s.magnetometer_timestamp_s,
            "alignment did not consume its magnetic seed");
    }
    if (tick < 40)
      continue;
    CHECK(
        covariance_positive_definite(s.errorCovariance),
        "stationary aiding covariance lost symmetry or positive definiteness");
    CHECK(s.status_zeroVelocityCorrectionAccepted ==
              (s.vehicleAtRest && stationary_configured),
          "declared rest starved, inferred from quiet IMU, or not released");
    CHECK(s.estimate_valid && s.status_predictionAccepted,
          "invalid hover prediction");
    CHECK(s.status_opticalFlowCorrectionAccepted == !bad_flow,
          "flow starvation or bad quality fused");
    CHECK(s.status_magnetometerCorrectionAccepted == auxiliary_new,
          "magnetic packet starved or fused twice");
    CHECK(s.status_barometerCorrectionAccepted == auxiliary_new,
          "barometer packet starved or fused twice");
    CHECK(s.status_acceptedCorrectionCount == count + 1,
          "fusion instant must increment exactly once");
    if (tick >= 60 && tick < 70) {
      CHECK(s.gpsConsecutiveRejections > 0,
            "supplemental fusion hid rejected GPS");
      CHECK(s.status_rejectionElapsed_s > 0,
            "supplemental fusion reset anchor clock");
    }
    if (bad_flow)
      CHECK(s.opticalFlowConsecutiveRejections > 0,
            "GPS acceptance hid rejected flow");
    if (tick >= 145 && tick < 150) {
      CHECK(s.status_anchorSource == 3 && s.status_correctionSource == 3,
            "flow acceptance was not attributed to the anchor");
      CHECK(s.status_rejectionElapsed_s < .015f,
            "accepted anchor triggered recovery clock");
    }
    for (int axis = 0; axis < 3; ++axis) {
      CHECK(isfinite(s.estimate_positionWorldEnu_m[axis]) &&
                fabsf(s.estimate_positionWorldEnu_m[axis]) < .01f,
            "hover drift");
      CHECK(s.errorCovariance[axis][axis] < 2,
            "stationary covariance inflated");
    }
  }
  const int count = s.status_acceptedCorrectionCount;
  NavigationEstimator_dostep(&s);
  const int tick = 200;
  CHECK(!s.status_opticalFlowCorrectionAccepted &&
            !s.status_barometerCorrectionAccepted &&
            !s.status_magnetometerCorrectionAccepted &&
            !s.status_gpsPositionCorrectionAccepted,
        "held packets fused again");
  CHECK(s.status_acceptedCorrectionCount == count,
        "duplicate delivery shifted fusion instant");
  s.imu_timestamp_s = 2.0f;
  s.vehicleAtRest = true;
  s.gps_valid = s.opticalFlow_valid = s.magnetometer_valid = s.barometer_valid =
      false;
  NavigationEstimator_dostep(&s);
  CHECK(!s.rumoca_galec_error_signal_status,
        "stationary-only generated C error");
  CHECK(s.status_zeroVelocityCorrectionAccepted == stationary_configured,
        "stationary-only update or invalid variance handling failed");
  CHECK(s.status_acceptedCorrectionCount == count + stationary_configured,
        "stationary-only correction did not notify the output predictor");
  NavigationEstimator_dostep(&s);
  CHECK(!s.status_zeroVelocityCorrectionAccepted,
        "duplicate IMU repeated the stationary constraint");
  CHECK(s.status_acceptedCorrectionCount == count + stationary_configured,
        "duplicate stationary correction notified the output predictor");
  s.vehicleAtRest = false;
  s.imu_timestamp_s = 2.01f;
  NavigationEstimator_dostep(&s);
  CHECK(!s.status_zeroVelocityCorrectionAccepted,
        "quiet IMU alone incorrectly declared rest");
  CHECK(s.status_acceptedCorrectionCount == count + stationary_configured,
        "released stationary constraint still notified a correction");
  puts("ESKF simultaneous aiding, GPS loss/return, and independent rejection "
       "health passed");
  return 0;
}

int main(void) {
  for (int semi_direct_bias = 0; semi_direct_bias <= 1; ++semi_direct_bias)
    for (int geometric_alignment = 0; geometric_alignment <= 1;
         ++geometric_alignment)
      for (int vector_fusion = 0; vector_fusion <= 1; ++vector_fusion)
        for (int stationary = 0; stationary <= 1; ++stationary)
          if (check_aiding(vector_fusion, geometric_alignment, stationary, .01f,
                           semi_direct_bias))
            return 1;
  const float invalid_variances[] = {0, -1, NAN, INFINITY};
  for (unsigned configuration = 0; configuration < 4; ++configuration)
    for (int semi_direct_bias = 0; semi_direct_bias <= 1; ++semi_direct_bias)
      if (check_aiding(false, false, true, invalid_variances[configuration],
                       semi_direct_bias))
        return 1;
  return 0;
}
