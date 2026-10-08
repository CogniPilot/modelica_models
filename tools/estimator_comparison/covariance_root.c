/* SPDX-License-Identifier: Apache-2.0 */
#include "Tests_CovarianceRootReplay.h"
#include <string.h>

int covariance_root(const float columns[450], float output[225]) {
  CovarianceRootReplayState state = {0};
  CovarianceRootReplay_startup(&state);
  memcpy(state.columns, columns, sizeof(state.columns));
  CovarianceRootReplay_dostep(&state);
  memcpy(output, state.root, sizeof(state.root));
  return state.rumoca_galec_error_signal_status;
}
