#include <stdio.h>

#include "Tests_VisualCouplingReplay.h"

static int read_values(float *values, int count)
{
    for (int component = 0; component < count; ++component)
        if (scanf("%f", &values[component]) != 1) return 0;
    return 1;
}

static void write_values(const float *values, int count)
{
    for (int component = 0; component < count; ++component)
        printf("%.9g ", values[component]);
}

int main(void)
{
    VisualCouplingReplayState state;
    for (;;) {
        VisualCouplingReplay_startup(&state);
        int first = scanf("%f", &state.quaternionWorldBody[0]);
        if (first == EOF) return 0;
        if (first != 1 || !read_values(&state.quaternionWorldBody[1], 3)
            || !read_values(state.positionWorld_m, 3)) return 2;
        for (int row = 0; row < 15; ++row)
            if (!read_values(state.priorCovariance[row], 15)) return 2;
        for (int row = 0; row < 4; ++row)
            if (!read_values(state.landmarksWorld_m[row], 3)) return 2;
        for (int row = 0; row < 4; ++row)
            if (!read_values(state.observations[row], 3)) return 2;
        for (int row = 0; row < 12; ++row)
            if (!read_values(state.measurementCovariance[row], 12)) return 2;
        for (int row = 0; row < 3; ++row)
            if (!read_values(state.rotationBodyCamera[row], 3)) return 2;
        if (!read_values(state.cameraPositionBody_m, 3)
            || !read_values(state.intrinsics, 4)) return 2;
        VisualCouplingReplay_dostep(&state);
        write_values(state.looseState, 16);
        write_values(state.tightState, 16);
        for (int row = 0; row < 15; ++row) write_values(state.looseCovariance[row], 15);
        for (int row = 0; row < 15; ++row) write_values(state.tightCovariance[row], 15);
        printf("%d %d %.9g %.9g %u\n", state.looseAccepted ? 1 : 0,
               state.tightAccepted ? 1 : 0, state.looseNis, state.tightNis,
               (unsigned)state.rumoca_galec_error_signal_status);
    }
}
