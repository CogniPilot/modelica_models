#ifndef ESTIMATOR_COMPARISON_TIMING_H
#define ESTIMATOR_COMPARISON_TIMING_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef struct {
  uint64_t total_ns, maximum_ns, calls;
} ReplayTiming;

static bool timing_enabled;
static ReplayTiming filter_timing, preintegration_timing, predictor_timing,
    queue_timing;

static uint64_t timing_now(void) {
  if (!timing_enabled)
    return 0;
  struct timespec instant;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &instant))
    abort();
  return (uint64_t)instant.tv_sec * 1000000000u + instant.tv_nsec;
}

static void timing_record(ReplayTiming *timing, uint64_t start_ns) {
  if (!timing_enabled)
    return;
  const uint64_t elapsed_ns = timing_now() - start_ns;
  timing->total_ns += elapsed_ns;
  if (elapsed_ns > timing->maximum_ns)
    timing->maximum_ns = elapsed_ns;
  ++timing->calls;
}

static void timing_write(FILE *output, const char *name,
                         const ReplayTiming *timing) {
  fprintf(output,
          "\"%s\":{\"calls\":%llu,\"total_ns\":%llu,\"maximum_ns\":%llu}", name,
          (unsigned long long)timing->calls,
          (unsigned long long)timing->total_ns,
          (unsigned long long)timing->maximum_ns);
}

#endif
