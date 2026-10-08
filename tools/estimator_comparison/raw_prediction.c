/* SPDX-License-Identifier: Apache-2.0 */
#include "Tests_RawPredictionReplay.h"
#include <string.h>

int raw_prediction(const float covariance[225], const float prior_root[225], float dt, int square_root,
                   float output[241]) {
  RawPredictionReplayState state = {0};
  RawPredictionReplay_startup(&state);
  memcpy(state.covariance, covariance, sizeof(state.covariance));
  memcpy(state.priorRoot, prior_root, sizeof(state.priorRoot));
  state.dt = dt;
  state.squareRoot = square_root != 0;
  RawPredictionReplay_dostep(&state);
  memcpy(output, state.nominal, sizeof(state.nominal));
  memcpy(output + 16, state.predictedCovariance, sizeof(state.predictedCovariance));
  return state.rumoca_galec_error_signal_status;
}
