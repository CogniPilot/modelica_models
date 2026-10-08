/* SPDX-License-Identifier: Apache-2.0 */
#include "Tests_SemiDirectBiasReplay.h"
#include <string.h>

int semi_direct_correction(const float nominal[16], const float correction[15],
                          float corrected[16], float reset[225]) {
  SemiDirectBiasReplayState state = {0};
  SemiDirectBiasReplay_startup(&state);
  memcpy(state.nominal, nominal, sizeof(state.nominal));
  memcpy(state.correction, correction, sizeof(state.correction));
  SemiDirectBiasReplay_dostep(&state);
  memcpy(corrected, state.corrected, sizeof(state.corrected));
  memcpy(reset, state.resetJacobian, sizeof(state.resetJacobian));
  return state.rumoca_galec_error_signal_status;
}
