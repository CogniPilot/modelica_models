#include <stdio.h>

#include "Tests_VisualLandmarkReplay.h"

static int read_values(float *values, int count)
{
    for (int component = 0; component < count; ++component)
        if (scanf("%f", &values[component]) != 1) return 0;
    return 1;
}

int main(void)
{
    VisualLandmarkReplayState state;
    for (;;) {
        VisualLandmarkReplay_startup(&state);
        int first = scanf("%f", &state.quaternionWorldBody[0]);
        if (first == EOF) return 0;
        if (first != 1 || !read_values(&state.quaternionWorldBody[1], 3)
            || !read_values(state.positionWorld_m, 3)
            || !read_values(state.landmarkWorld_m, 3)) return 2;
        for (int row = 0; row < 3; ++row)
            if (!read_values(state.rotationBodyCamera[row], 3)) return 2;
        if (!read_values(state.cameraPositionBody_m, 3)
            || !read_values(state.intrinsics, 4)
            || !read_values(state.correction, 15)) return 2;
        VisualLandmarkReplay_dostep(&state);
        for (int component = 0; component < 3; ++component)
            printf("%.9g ", state.expected[component]);
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 15; ++column)
                printf("%.9g ", state.navigationJacobian[row][column]);
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                printf("%.9g ", state.landmarkJacobian[row][column]);
        printf("%d %u\n", state.valid ? 1 : 0,
               (unsigned)state.rumoca_galec_error_signal_status);
    }
}
