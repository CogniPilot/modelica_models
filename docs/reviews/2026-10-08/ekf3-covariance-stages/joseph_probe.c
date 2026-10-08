#include "Tests_BarometerConsiderReplay.c"
#include <string.h>
int native_gain_joseph(float factor[15][15], float covariance[15][15],
                      float gain[15][1], float noise[1][1], float output[225]) {
    BarometerConsiderReplayState state = {0};
    josephUpdate_specialization_53(&state, factor, covariance, gain, noise);
    memcpy(output,
        state.rumoca_galec_scratch.rumoca_galec_g3.josephUpdate_specialization_53.covarianceNext,
        225 * sizeof(float));
    return state.rumoca_galec_error_signal_status;
}
