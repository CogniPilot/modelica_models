/* SPDX-License-Identifier: Apache-2.0 */
#include "Vehicles_Rdd2_NavigationEstimator.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static NavigationEstimatorState estimator;

#define CHECK(condition, message)                                               \
  do {                                                                         \
    if (!(condition)) {                                                        \
      fprintf(stderr, "%s\n", message);                                       \
      return 1;                                                                \
    }                                                                          \
  } while (0)

static void startup(void) {
  memset(&estimator, 0, sizeof(estimator));
  NavigationEstimator_startup(&estimator);
  estimator.useDeclaredRestBarometerCalibration = true;
  estimator.barometerBiasCalibrationSamples = 2;
  estimator.barometerBiasProcessNoise_m2_s = 0.1f;
  estimator.initialAlignmentWindow_s = 0;
  estimator.initialAlignmentTimeout_s = 0;
  estimator.pseudoPositionVariance_m2 = 0;
  estimator.zeroVelocityVariance_m2_s2 = 0;
  estimator.imu_valid = estimator.imu_fresh = true;
  estimator.imu_integrationTime_s = 0.01f;
  estimator.specificForceBodyFlu_m_s2[2] = 9.81f;
  estimator.deltaVelocityBodyFlu_m_s[2] = 0.0981f;
  estimator.deltaPositionBodyFlu_m[2] = 0.0004905f;
  estimator.deltaQuaternionBodyFlu[0] = 1;
  estimator.barometer_valid = estimator.barometer_fresh = true;
  estimator.altitudeWorldEnu_m = 0.2f;
  estimator.variance_m2 = 0.01f;
}

static int tick(float time, bool at_rest) {
  estimator.imu_timestamp_s = time;
  estimator.barometer_timestamp_s = time;
  estimator.vehicleAtRest = at_rest;
  NavigationEstimator_dostep(&estimator);
  CHECK(!estimator.rumoca_galec_error_signal_status,
        "Datum fixture generated a runtime error");
  CHECK(isfinite(estimator.barometerBias_m) &&
            estimator.barometerBiasVariance_m2 > 0,
        "Datum calibration lost a finite mean or positive variance");
  return 0;
}

int main(void) {
  startup();
  for (unsigned sample = 0; sample < 5; ++sample) {
    CHECK(!tick(sample * 0.01f, true), "Startup rest update failed");
    if (estimator.barometerBiasCalibrationCount != (int)sample + 1)
      fprintf(stderr, "sample %u: count=%d closed=%d update=%d bias=%g\n",
              sample, estimator.barometerBiasCalibrationCount,
              estimator.barometerBiasCalibrationClosed,
              estimator.barometerBiasUpdateAccepted, estimator.barometerBias_m);
    CHECK(estimator.barometerBiasCalibrationCount == (int)sample + 1,
          "Declared rest stopped calibrating at the minimum sample count");
    CHECK(!estimator.status_barometerCorrectionAccepted,
          "A calibration pressure packet was also fused into navigation");
  }
  const float learned_bias = estimator.barometerBias_m;
  estimator.imu_timestamp_s = 0.05f;
  NavigationEstimator_dostep(&estimator);
  CHECK(estimator.barometerBiasCalibrationCount == 5 &&
            !estimator.barometerBiasUpdateAccepted &&
            !estimator.status_barometerCorrectionAccepted,
        "A duplicate pressure timestamp was assimilated again");
  CHECK(!tick(0.06f, false), "Rest release failed");
  CHECK(estimator.barometerBiasCalibrationClosed &&
            estimator.status_barometerCorrectionAccepted,
        "Releasing startup rest did not close calibration and enable fusion");
  estimator.altitudeWorldEnu_m = 2;
  CHECK(!tick(0.07f, false), "Moving pressure packet failed");
  CHECK(!tick(0.08f, true), "Later rest packet failed");
  CHECK(estimator.barometerBias_m == learned_bias &&
            estimator.barometerBiasCalibrationCount == 5,
        "Later rest at another altitude restarted the startup datum");
  estimator.reset = true;
  CHECK(!tick(0.09f, true), "Datum reset failed");
  CHECK(estimator.barometerBiasCalibrationCount == 0 &&
            !estimator.barometerBiasCalibrationClosed,
        "Reset did not reopen the startup calibration epoch");
  estimator.reset = false;
  CHECK(!tick(0.10f, true), "Calibration after reset failed");
  CHECK(estimator.barometerBiasCalibrationCount == 1,
        "New rest epoch did not assimilate pressure after reset");

  startup();
  estimator.barometerBiasCalibrationSamples = 5;
  CHECK(!tick(0, true), "Short startup rest failed");
  CHECK(!tick(0.01f, false), "Early takeoff failed");
  CHECK(!tick(0.02f, false), "Pressure after early takeoff failed");
  CHECK(estimator.barometerBiasCalibrationCount == 1 &&
            estimator.barometerBiasInitialized &&
            estimator.status_barometerCorrectionAccepted,
        "Early takeoff left pressure fusion waiting for stationary samples");

  startup();
  CHECK(!tick(0, false), "Startup without declared rest failed");
  const float initial_variance = estimator.barometerBiasVariance_m2;
  CHECK(!tick(0.01f, false), "Pressure without startup calibration failed");
  CHECK(!tick(0.02f, true), "Later rest without startup calibration failed");
  CHECK(estimator.barometerBiasCalibrationCount == 0 &&
            estimator.barometerBias_m == estimator.initialBarometerBias_m &&
            estimator.barometerBiasVariance_m2 > initial_variance,
        "Absent startup rest changed the datum or discarded its process noise");

  startup();
  estimator.altitudeWorldEnu_m = NAN;
  estimator.vehicleAtRest = true;
  NavigationEstimator_dostep(&estimator);
  CHECK(isfinite(estimator.barometerBias_m) &&
            estimator.barometerBiasCalibrationCount == 0,
        "A nonfinite startup pressure poisoned the datum");
  puts("Declared-rest pressure calibration, withholding, duplicates, early "
       "takeoff, reset and absent rest passed");
  return 0;
}
