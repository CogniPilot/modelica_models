#pragma once

#include <cstdio>
#include <cstdlib>

inline void writeNativeImuIntegrity(const char *stage, const double (&values)[25]) {
  static FILE *stream = nullptr;
  static bool opened = false;
  if (!opened) {
    opened = true;
    const char *path = std::getenv("NATIVE_IMU_INTEGRITY_PATH");
    if (!path) return;
    stream = std::fopen(path, "wx");
    if (!stream) std::abort();
    std::fprintf(stream, "stage,publication_us,fusion_us,imu_sample_ms,gps_sample_ms,"
                         "baro_sample_ms,height_error_m,vertical_velocity_error_m_s,"
                         "height_observation_variance,velocity_observation_variance,"
                         "threshold_multiplier,time_since_bad_ms,last_bad_ms,"
                         "last_good_ms,bad_imu_data,estimated_velocity_down_m_s,"
                         "gps_velocity_down_m_s,observed_velocity_down_m_s,"
                         "estimated_position_down_m,observed_position_down_m,"
                         "on_ground,takeoff_expected,touchdown_expected,"
                         "aiding_mode,height_source,gps_data_to_fuse\n");
  }
  if (!stream) return;
  std::fprintf(stream, "%s", stage);
  for (const double value : values) std::fprintf(stream, ",%.17g", value);
  std::fputc('\n', stream);
}
