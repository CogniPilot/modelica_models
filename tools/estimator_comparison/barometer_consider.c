/* SPDX-License-Identifier: Apache-2.0 */
#include "Tests_BarometerConsiderReplay.h"
#include <math.h>
#include <string.h>

int barometer_consider(const float nominal[16], const float covariance[225],
                      const float cross[15], const float observation[109],
                      const float transition[225], const float bounds[15],
                      const float pressure[6], int mode, int operation,
                      float output[261]) {
  BarometerConsiderReplayState state = {0};
  BarometerConsiderReplay_startup(&state);
  memcpy(state.priorState, nominal, sizeof(state.priorState));
  memcpy(state.priorCovariance, covariance, sizeof(state.priorCovariance));
  memcpy(state.priorCrossCovariance, cross, sizeof(state.priorCrossCovariance));
  for (unsigned row = 0; row < 15; ++row)
    for (unsigned column = 0; column <= row; ++column) {
      double value = covariance[15 * row + column];
      for (unsigned term = 0; term < column; ++term)
        value -= (double)state.priorRoot[row][term] * state.priorRoot[column][term];
      if (row == column) {
        if (value <= 0)
          return 2;
        state.priorRoot[row][column] = sqrt(value);
      } else {
        state.priorRoot[row][column] = value / state.priorRoot[column][column];
      }
    }
  memcpy(state.measurementResidual, observation, sizeof(state.measurementResidual));
  memcpy(state.observationMatrix, observation + 3, sizeof(state.observationMatrix));
  memcpy(state.measurementCovariance, observation + 48, sizeof(state.measurementCovariance));
  memcpy(state.measurementStateCrossCovariance, observation + 57,
         sizeof(state.measurementStateCrossCovariance));
  memcpy(state.attitudeAxis, observation + 102, sizeof(state.attitudeAxis));
  state.innovationGate = observation[105];
  memcpy(state.measurementBarometerCrossCovariance, observation + 106,
         sizeof(state.measurementBarometerCrossCovariance));
  memcpy(state.transition, transition, sizeof(state.transition));
  memcpy(state.varianceBounds, bounds, sizeof(state.varianceBounds));
  state.biasVariance = pressure[0];
  state.biasProcessNoise = pressure[1];
  state.pressureAltitude = pressure[2];
  state.pressureVariance = pressure[3];
  state.measurementAge = pressure[4];
  state.biasMean = pressure[5];
  state.useSemiDirectBias = (mode & 1) != 0;
  state.headingOnly = (mode & 2) != 0;
  state.useSquareRoot = (mode & 4) != 0;
  state.useJointBarometerBias = (mode & 8) != 0;
  state.operation = operation;
  BarometerConsiderReplay_dostep(&state);
  memcpy(output, state.correctedState, sizeof(state.correctedState));
  memcpy(output + 16, state.posteriorCovariance, sizeof(state.posteriorCovariance));
  memcpy(output + 241, state.posteriorCrossCovariance,
         sizeof(state.posteriorCrossCovariance));
  output[256] = state.nis;
  output[257] = state.reason;
  output[258] = state.accepted;
  output[259] = state.posteriorBiasMean;
  output[260] = state.posteriorBiasVariance;
  return state.rumoca_galec_error_signal_status;
}
