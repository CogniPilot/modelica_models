#pragma once

#include <cstdio>
#include <cstdlib>

template <typename Filter>
void writeNativeHeightState(double publication_s, const Filter &filter, double) {
  static FILE *stream = nullptr;
  static bool opened = false;
  if (!opened) {
    opened = true;
    const char *path = std::getenv("NATIVE_HEIGHT_STATE_PATH");
    if (!path) return;
    stream = std::fopen(path, "wx");
    if (!stream) std::abort();
    std::fprintf(stream, "publication_s,range_height,range_terrain,flow_terrain,"
                         "barometer_height,gps_height,height_reference,"
                         "height_above_ground_m,terrain_variance_m2,in_air,"
                         "range_kinematic_consistent,range_fault\n");
  }
  if (!stream) return;
  const auto flags = filter.control_status_flags();
  std::fprintf(stream, "%.17g,%d,%d,%d,%d,%d,%d,%.17g,%.17g,%d,%d,%d\n",
               publication_s, (int)flags.rng_hgt, (int)flags.rng_terrain,
               (int)flags.opt_flow_terrain, (int)flags.baro_hgt,
               (int)flags.gps_hgt, (int)filter.getHeightSensorRef(),
               (double)filter.getHagl(), (double)filter.getTerrainVariance(),
               (int)flags.in_air, (int)flags.rng_kin_consistent,
               (int)flags.rng_fault);
}
