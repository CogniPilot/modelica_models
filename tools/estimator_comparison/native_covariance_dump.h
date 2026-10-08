#pragma once

#include <cstdio>
#include <cstdlib>

template <typename Covariance>
static void writeNativeCovariance(unsigned long long publication_us,
                                  unsigned long long fusion_us,
                                  double bias_period_s, const double state[16],
                                  const Covariance &covariance) {
  static FILE *stream = nullptr;
  static bool opened = false;
  if (!opened) {
    opened = true;
    const char *path = std::getenv("NATIVE_COVARIANCE_PATH");
    if (!path) return;
    stream = std::fopen(path, "wx");
    if (!stream) std::abort();
    std::fprintf(stream, "publication_us,fusion_us,bias_period_s");
    for (unsigned component = 0; component < 16; ++component)
      std::fprintf(stream, ",s%u", component);
    for (unsigned row = 0; row < 24; ++row)
      for (unsigned column = 0; column < 24; ++column)
        std::fprintf(stream, ",p%u_%u", row, column);
    std::fputc('\n', stream);
  }
  if (!stream) return;
  std::fprintf(stream, "%llu,%llu,%.17g", publication_us, fusion_us,
               bias_period_s);
  for (unsigned component = 0; component < 16; ++component)
    std::fprintf(stream, ",%.17g", state[component]);
  for (unsigned row = 0; row < 24; ++row)
    for (unsigned column = 0; column < 24; ++column)
      std::fprintf(stream, ",%.17g", static_cast<double>(covariance(row, column)));
  std::fputc('\n', stream);
}
