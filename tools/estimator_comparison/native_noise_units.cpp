/* SPDX-License-Identifier: Apache-2.0 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ekf_derivation/generated/predict_covariance.h>

int main(int argc, char **argv) {
  if (argc != 1 && argc != 3)
    return 2;
  float density[2]{};
  if (argc == 3) {
    for (unsigned sensor = 0; sensor < 2; ++sensor) {
      char *end;
      density[sensor] = strtof(argv[sensor + 1], &end);
      if (*end || !std::isfinite(density[sensor]) || density[sensor] < 1e-9f ||
          density[sensor] > 100)
        return 2;
    }
  }
  matrix::Matrix<float, 25, 1> state{};
  matrix::Matrix<float, 24, 24> prior{};
  matrix::Matrix<float, 3, 1> zero{}, acceleration_variance{};
  state(0, 0) = 1;
  const float intervals[] = {.005f, .01f, .0125f, .02f};
  for (float interval : intervals) {
    const float gyro_noise = argc == 3 ? density[0] / sqrtf(interval) : .015f;
    const float accel_noise = argc == 3 ? density[1] / sqrtf(interval) : .35f;
    for (unsigned axis = 0; axis < 3; ++axis)
      acceleration_variance(axis, 0) = accel_noise * accel_noise;
    const auto covariance =
        sym::PredictCovariance(state, prior, zero, acceleration_variance, zero,
                               gyro_noise * gyro_noise, interval);
    const float angle = covariance(0, 0), velocity = covariance(3, 3);
    const float expected_angle =
        argc == 3 ? density[0] * density[0] * interval
                  : gyro_noise * gyro_noise * interval * interval;
    const float expected_velocity =
        argc == 3 ? density[1] * density[1] * interval
                  : accel_noise * accel_noise * interval * interval;
    if (fabsf(angle / expected_angle - 1) > 1e-5f ||
        fabsf(velocity / expected_velocity - 1) > 1e-5f)
      return 1;
    printf("%.9g,%.9g,%.9g\n", interval, angle, velocity);
  }
}
