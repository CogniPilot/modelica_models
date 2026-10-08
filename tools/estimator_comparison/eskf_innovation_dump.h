#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static struct {
  const char *sensor;
  unsigned dimension;
  float sample_s, age_s, gate;
  bool computed;
  float residual[6], noise_diagonal[6], innovation[6][6];
} eskf_observation;

static void beginEskfInnovation(const char *sensor, unsigned dimension,
                                float sample_s, float age_s, float gate) {
  memset(&eskf_observation, 0, sizeof(eskf_observation));
  eskf_observation.sensor = sensor;
  eskf_observation.dimension = dimension;
  eskf_observation.sample_s = sample_s;
  eskf_observation.age_s = age_s;
  eskf_observation.gate = gate;
}

static void captureEskfInnovation(unsigned dimension, const float residual[],
                                  const float *noise, const float *innovation) {
  if (!eskf_observation.sensor) return;
  if (dimension != eskf_observation.dimension || dimension > 6) abort();
  eskf_observation.computed = true;
  for (unsigned row = 0; row < dimension; ++row) {
    eskf_observation.residual[row] = residual[row];
    eskf_observation.noise_diagonal[row] = noise[row * dimension + row];
    for (unsigned column = 0; column < dimension; ++column)
      eskf_observation.innovation[row][column] = innovation[row * dimension + column];
  }
}

static void finishEskfInnovation(bool accepted, int outcome, float nis) {
  static FILE *stream = NULL;
  static bool opened = false;
  if (!opened) {
    opened = true;
    const char *path = getenv("ESKF_INNOVATION_PATH");
    if (path) {
      stream = fopen(path, "wx");
      if (!stream) abort();
      fputs("sensor,state_epoch_s,sample_epoch_s,age_s,dimension,computed,accepted,outcome,nis,gate", stream);
      for (unsigned axis = 0; axis < 6; ++axis) fprintf(stream, ",r%u", axis);
      for (unsigned axis = 0; axis < 6; ++axis) fprintf(stream, ",noise%u", axis);
      for (unsigned row = 0; row < 6; ++row)
        for (unsigned column = 0; column < 6; ++column)
          fprintf(stream, ",s%u_%u", row, column);
      fputc('\n', stream);
    }
  }
  if (stream) {
    if (accepted && !eskf_observation.computed) abort();
    fprintf(stream, "%s,%.17g,%.17g,%.17g,%u,%d,%d,%d,%.17g,%.17g",
            eskf_observation.sensor,
            (double)(eskf_observation.sample_s + eskf_observation.age_s),
            (double)eskf_observation.sample_s, (double)eskf_observation.age_s,
            eskf_observation.dimension, (int)eskf_observation.computed,
            (int)accepted, outcome, (double)nis, (double)eskf_observation.gate);
    for (unsigned axis = 0; axis < 6; ++axis)
      fprintf(stream, ",%.17g", (double)eskf_observation.residual[axis]);
    for (unsigned axis = 0; axis < 6; ++axis)
      fprintf(stream, ",%.17g", (double)eskf_observation.noise_diagonal[axis]);
    for (unsigned row = 0; row < 6; ++row)
      for (unsigned column = 0; column < 6; ++column)
        fprintf(stream, ",%.17g", (double)eskf_observation.innovation[row][column]);
    fputc('\n', stream);
  }
  eskf_observation.sensor = NULL;
}
