#pragma once

#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>

static unsigned long long nativeCovarianceStageBound(const char *name,
                                                     unsigned long long fallback) {
  const char *value = std::getenv(name);
  if (!value) return fallback;
  if (*value < '0' || *value > '9') std::abort();
  char *end = nullptr;
  errno = 0;
  const auto bound = std::strtoull(value, &end, 10);
  if (errno || *end) std::abort();
  return bound;
}

template <typename Scalar, typename Covariance>
inline void writeNativeCovarianceStage(
    unsigned long long publication_us, unsigned long long fusion_us,
    double bias_period_s, const char *stage, int axis, unsigned state_index_limit,
    bool gyro_inhibited, bool accel_inhibited, const bool accel_axis_inhibited[3],
    bool bad_imu_data, unsigned aiding_mode,
    const Scalar state[24], const Scalar *gain, double noise_variance,
    const Covariance &covariance) {
  static FILE *stream = nullptr;
  static bool opened = false;
  static unsigned long long start_us = 0, end_us = ULLONG_MAX;
  if (!opened) {
    opened = true;
    const char *path = std::getenv("NATIVE_COVARIANCE_STAGE_PATH");
    if (!path) return;
    start_us = nativeCovarianceStageBound("NATIVE_COVARIANCE_STAGE_START_US", 0);
    end_us = nativeCovarianceStageBound("NATIVE_COVARIANCE_STAGE_END_US", ULLONG_MAX);
    if (end_us < start_us) std::abort();
    stream = std::fopen(path, "wx");
    if (!stream) std::abort();
    std::fprintf(stream, "publication_us,fusion_us,bias_period_s,stage,axis,"
                         "state_index_limit,gyro_inhibited,accel_inhibited,"
                         "accel_x_inhibited,accel_y_inhibited,accel_z_inhibited,"
                         "bad_imu_data,aiding_mode,noise_variance");
    for (unsigned component = 0; component < 24; ++component)
      std::fprintf(stream, ",s%u", component);
    for (unsigned component = 0; component < 24; ++component)
      std::fprintf(stream, ",k%u", component);
    for (unsigned row = 0; row < 24; ++row)
      for (unsigned column = 0; column < 24; ++column)
        std::fprintf(stream, ",p%u_%u", row, column);
    std::fputc('\n', stream);
  }
  if (!stream || publication_us < start_us || publication_us > end_us) return;
  std::fprintf(stream, "%llu,%llu,%.17g,%s,%d,%u,%d,%d,%d,%d,%d,%d,%u,%.17g",
               publication_us, fusion_us, bias_period_s, stage, axis,
               state_index_limit, gyro_inhibited, accel_inhibited,
               accel_axis_inhibited[0], accel_axis_inhibited[1],
               accel_axis_inhibited[2], bad_imu_data, aiding_mode, noise_variance);
  for (unsigned component = 0; component < 24; ++component)
    std::fprintf(stream, ",%.17g", static_cast<double>(state[component]));
  for (unsigned component = 0; component < 24; ++component)
    std::fprintf(stream, ",%.17g", gain && component <= state_index_limit
                                    ? static_cast<double>(gain[component]) : 0.0);
  for (unsigned row = 0; row < 24; ++row)
    for (unsigned column = 0; column < 24; ++column)
      std::fprintf(stream, ",%.17g", static_cast<double>(covariance[row][column]));
  std::fputc('\n', stream);
}

#define OBSERVE_EKF3_COVARIANCE(stage, axis, gain, noise)                    \
  do {                                                                     \
    if (core_index == 0)                                                    \
      writeNativeCovarianceStage(                                          \
          dal.micros64(),                                                   \
          static_cast<unsigned long long>(imuDataDelayed.time_ms) * 1000,   \
          dtEkfAvg, stage, axis, stateIndexLim, inhibitDelAngBiasStates,      \
          inhibitDelVelBiasStates, dvelBiasAxisInhibit, badIMUdata,          \
          static_cast<unsigned>(PV_AidingMode), statesArray, gain,         \
          noise, P);                                                       \
  } while (false)
