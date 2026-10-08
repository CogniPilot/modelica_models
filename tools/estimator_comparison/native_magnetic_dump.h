#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>

template <typename Filter>
void writeNativeMagneticState(double publication_s, const Filter &filter,
                             double configured_declination_deg) {
  static FILE *stream = nullptr;
  static bool opened = false;
  if (!opened) {
    opened = true;
    const char *path = std::getenv("NATIVE_MAG_STATE_PATH");
    if (!path) return;
    stream = std::fopen(path, "wx");
    if (!stream) std::abort();
    std::fprintf(stream, "publication_s,earth_n_gauss,earth_e_gauss,earth_d_gauss,"
                         "bias_x_gauss,bias_y_gauss,bias_z_gauss,wmm_declination_deg,"
                         "wmm_inclination_deg,measured_inclination_deg,"
                         "measured_strength_gauss,wmm_strength_gauss,mag_aligned,"
                         "mag_3D,mag_heading,mag_declination,in_air,"
                         "configured_declination_deg\n");
  }
  if (!stream) return;
  const auto earth = filter.getMagEarthField();
  const auto bias = filter.getMagBias();
  const auto flags = filter.control_status_flags();
  float declination = NAN, inclination = NAN;
  float measured_inclination, inclination_reference, measured_strength, strength_reference;
  filter.get_mag_decl_deg(declination);
  filter.get_mag_inc_deg(inclination);
  filter.get_mag_checks(measured_inclination, inclination_reference,
                        measured_strength, strength_reference);
  std::fprintf(stream, "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
                       "%.17g,%.17g,%.17g,%.17g,%d,%d,%d,%d,%d,%.17g\n",
               publication_s, (double)earth(0), (double)earth(1), (double)earth(2),
               (double)bias(0), (double)bias(1), (double)bias(2), (double)declination,
               (double)inclination, (double)measured_inclination,
               (double)measured_strength, (double)strength_reference,
               (int)flags.mag_aligned_in_flight, (int)flags.mag_3D,
               (int)flags.mag_hdg, (int)flags.mag_dec, (int)flags.in_air,
               configured_declination_deg);
}
