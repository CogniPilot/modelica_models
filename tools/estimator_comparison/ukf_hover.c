/* SPDX-License-Identifier: Apache-2.0 */
#include "Tests_UKFHoverCodegen.h"
#include <math.h>
#include <stdio.h>

static UKFHoverCodegenState state;

int main(void) {
    for (int mode = 0; mode < 2; ++mode) {
        UKFHoverCodegen_startup(&state);
        state.preintegrated = mode;
        for (int step = 0; step < 10; ++step) {
            state.first = step == 0;
            UKFHoverCodegen_dostep(&state);
            if (!state.success || state.rumoca_galec_error_signal_status) return 1;
            for (int i = 0; i < 3; ++i) {
                if (!isfinite(state.position[i]) || !isfinite(state.velocity[i])
                    || fabsf(state.position[i]) > 2e-5f
                    || fabsf(state.velocity[i]) > 2e-5f) {
                    fprintf(stderr, "UKF hover failed: mode=%d step=%d axis=%d p=%g v=%g\n",
                            mode, step, i, state.position[i], state.velocity[i]);
                    return 1;
                }
            }
        }
    }
    puts("UKF generated C preserved hover for raw and preintegrated prediction");
    return 0;
}
