#include <stdio.h>

#include "Tests_VisualTangentReplay.h"

int main(void)
{
    VisualTangentReplayState state;
    for (;;) {
        VisualTangentReplay_startup(&state);
        int first = scanf("%f", &state.quaternionWorldBody[0]);
        if (first == EOF) return 0;
        if (first != 1) return 2;
        for (int component = 1; component < 4; ++component)
            if (scanf("%f", &state.quaternionWorldBody[component]) != 1) return 2;
        for (int component = 0; component < 15; ++component)
            if (scanf("%f", &state.correction[component]) != 1) return 2;
        VisualTangentReplay_dostep(&state);
        if (state.rumoca_galec_error_signal_status != 0) return 3;
        for (int row = 0; row < 15; ++row)
            for (int column = 0; column < 15; ++column)
                printf("%.9g ", state.transform[row][column]);
        for (int component = 0; component < 16; ++component)
            printf("%.9g%c", state.injected[component], component == 15 ? '\n' : ' ');
    }
}
