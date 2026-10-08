#include <stdio.h>

#include "Tests_SyntheticLandmarkReplay.h"

static int read_values(float *values, int count)
{
    for (int component = 0; component < count; ++component)
        if (scanf("%f", &values[component]) != 1) return 0;
    return 1;
}

int main(void)
{
    SyntheticLandmarkReplayState state;
    for (;;) {
        SyntheticLandmarkReplay_startup(&state);
        int first = scanf("%f", &state.positionWorld_m[0]);
        if (first == EOF) return 0;
        if (first != 1 || !read_values(&state.positionWorld_m[1], 2)
            || !read_values(state.quaternionWorldBody, 4)) return 2;
        for (int landmark = 0; landmark < 4; ++landmark)
            if (!read_values(state.landmarksWorld_m[landmark], 3)) return 2;
        for (int row = 0; row < 3; ++row)
            if (!read_values(state.rotationBodyCamera[row], 3)) return 2;
        if (!read_values(state.cameraPositionBody_m, 3) || !read_values(state.intrinsics, 4)
            || scanf("%d %d", &state.imageSize_pixels[0], &state.imageSize_pixels[1]) != 2
            || !read_values(state.depthRange_m, 2)) return 2;
        for (int landmark = 0; landmark < 4; ++landmark)
            if (!read_values(state.measurementError[landmark], 3)) return 2;
        for (int landmark = 0; landmark < 4; ++landmark) {
            int detected;
            if (scanf("%d", &detected) != 1) return 2;
            state.detected[landmark] = detected != 0;
        }
        int available;
        if (scanf("%d", &available) != 1) return 2;
        state.available = available != 0;
        SyntheticLandmarkReplay_dostep(&state);
        for (int landmark = 0; landmark < 4; ++landmark)
            for (int component = 0; component < 3; ++component)
                printf("%.9g ", state.observations[landmark][component]);
        for (int landmark = 0; landmark < 4; ++landmark)
            printf("%d ", state.visible[landmark] ? 1 : 0);
        for (int landmark = 0; landmark < 4; ++landmark)
            for (int component = 0; component < 3; ++component)
                printf("%.9g ", state.grid[landmark][component]);
        for (int landmark = 0; landmark < 12; ++landmark)
            for (int component = 0; component < 3; ++component)
                printf("%.9g ", state.defaultObservations[landmark][component]);
        for (int landmark = 0; landmark < 12; ++landmark)
            printf("%d ", state.defaultVisible[landmark] ? 1 : 0);
        for (int landmark = 0; landmark < 12; ++landmark)
            printf("%d ", state.defaultLandmarkIds[landmark]);
        printf("%u\n", (unsigned)state.rumoca_galec_error_signal_status);
    }
}
