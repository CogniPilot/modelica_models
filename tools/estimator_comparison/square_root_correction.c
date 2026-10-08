/* SPDX-License-Identifier: Apache-2.0 */
#include "Tests_SquareRootCorrectionReplay.h"
#include <math.h>
#include <string.h>

int semi_direct_update(const float nominal[16], const float covariance[225],
                       const float observation[106], int mode,
                       float output[244]) {
  SquareRootCorrectionReplayState state = {0};
  SquareRootCorrectionReplay_startup(&state);
  memcpy(state.priorState, nominal, sizeof(state.priorState));
  memcpy(state.priorCovariance, covariance, sizeof(state.priorCovariance));
  for (unsigned row = 0; row < 15; ++row)
    for (unsigned column = 0; column <= row; ++column) {
      double value = covariance[15 * row + column];
      for (unsigned term = 0; term < column; ++term)
        value -=
            (double)state.priorRoot[row][term] * state.priorRoot[column][term];
      if (row == column) {
        if (value <= 0)
          return 2;
        state.priorRoot[row][column] = sqrt(value);
      } else {
        state.priorRoot[row][column] = value / state.priorRoot[column][column];
      }
    }
  memcpy(state.measurementResidual, observation,
         sizeof(state.measurementResidual));
  memcpy(state.observationMatrix, observation + 3,
         sizeof(state.observationMatrix));
  memcpy(state.measurementCovariance, observation + 48,
         sizeof(state.measurementCovariance));
  memcpy(state.measurementStateCrossCovariance, observation + 57,
         sizeof(state.measurementStateCrossCovariance));
  memcpy(state.attitudeAxis, observation + 102, sizeof(state.attitudeAxis));
  state.innovationGate = observation[105];
  state.useSemiDirectBias = (mode & 1) != 0;
  state.headingOnly = (mode & 2) != 0;
  SquareRootCorrectionReplay_dostep(&state);
  memcpy(output, state.correctedState, sizeof(state.correctedState));
  memcpy(output + 16, state.posteriorCovariance,
         sizeof(state.posteriorCovariance));
  output[241] = state.nis;
  output[242] = state.reason;
  output[243] = state.accepted;
  return state.rumoca_galec_error_signal_status;
}
