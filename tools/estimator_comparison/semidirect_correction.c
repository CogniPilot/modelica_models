/* SPDX-License-Identifier: Apache-2.0 */
#include "Tests_SemiDirectBiasCorrectionReplay.h"
#include <string.h>

int semi_direct_update(const float nominal[16], const float covariance[225],
                       const float observation[106], int mode,
                       float output[244]) {
  SemiDirectBiasCorrectionReplayState state = {0};
  SemiDirectBiasCorrectionReplay_startup(&state);
  memcpy(state.priorState, nominal, sizeof(state.priorState));
  memcpy(state.priorCovariance, covariance, sizeof(state.priorCovariance));
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
  SemiDirectBiasCorrectionReplay_dostep(&state);
  memcpy(output, state.correctedState, sizeof(state.correctedState));
  memcpy(output + 16, state.posteriorCovariance,
         sizeof(state.posteriorCovariance));
  output[241] = state.nis;
  output[242] = state.reason;
  output[243] = state.accepted;
  return state.rumoca_galec_error_signal_status;
}
