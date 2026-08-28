/* Host driver for the generated fusion-horizon output predictor.
 * Warms the ring so the horizon is full and the cadence phase is known, resets
 * the coverage counters, then runs exactly ONE tick of a chosen kind. The
 * counters therefore describe a single tick of that branch.
 *
 * Cadence, from the block: a release boundary falls on ticks 8k+1, and the
 * horizon is ready once the ring holds horizonEntries + deltasPerFusion = 168
 * entries. So warm=200 puts the measured tick on a boundary and warm=201 puts
 * it between boundaries, with the buffer full either way. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Estimation_FusionHorizon_OutputPredictor.h"

extern void __gcov_reset(void);
extern void __gcov_dump(void);

static OutputPredictorState g_state;

/* A coning-rich and sculling-rich stream, so no cross term is left at zero and
 * no branch is chosen by a degenerate input. */
static void feed(OutputPredictorState *s, int k, int shifted) {
    float t = 0.00125f * (float)k;
    s->reset = false;
    s->angularVelocityMeasuredBodyFlu_rad_s[0] = 0.6f * (float)sin(2.0 * 3.14159265 * 30.0 * t);
    s->angularVelocityMeasuredBodyFlu_rad_s[1] = 0.6f * (float)cos(2.0 * 3.14159265 * 30.0 * t);
    s->angularVelocityMeasuredBodyFlu_rad_s[2] = 0.35f;
    s->specificForceMeasuredBodyFlu_m_s2[0] = 1.5f * (float)sin(2.0 * 3.14159265 * 17.0 * t);
    s->specificForceMeasuredBodyFlu_m_s2[1] = -1.1f * (float)cos(2.0 * 3.14159265 * 23.0 * t);
    s->specificForceMeasuredBodyFlu_m_s2[2] = 9.81f + 0.8f * (float)sin(2.0 * 3.14159265 * 11.0 * t);
    s->horizonStateValid = true;
    s->horizonStateShifted = shifted ? true : false;
    s->horizonPositionWorldEnu_m[0] = 0.01f; s->horizonPositionWorldEnu_m[1] = -0.02f; s->horizonPositionWorldEnu_m[2] = 0.03f;
    s->horizonVelocityWorldEnu_m_s[0] = 0.4f; s->horizonVelocityWorldEnu_m_s[1] = -0.2f; s->horizonVelocityWorldEnu_m_s[2] = 0.05f;
    s->horizonQuaternionWorldBody[0] = 0.99619f; s->horizonQuaternionWorldBody[1] = 0.0436f;
    s->horizonQuaternionWorldBody[2] = -0.0523f; s->horizonQuaternionWorldBody[3] = 0.0611f;
    s->horizonGyroscopeBiasBodyFlu_rad_s[0] = 1.0e-3f;
    s->horizonGyroscopeBiasBodyFlu_rad_s[1] = -2.0e-3f;
    s->horizonGyroscopeBiasBodyFlu_rad_s[2] = 1.5e-3f;
    s->horizonAccelerometerBiasBodyFlu_m_s2[0] = 2.0e-2f;
    s->horizonAccelerometerBiasBodyFlu_m_s2[1] = -1.0e-2f;
    s->horizonAccelerometerBiasBodyFlu_m_s2[2] = 3.0e-2f;
}

static void report(const char *tag) {
    printf("%s count=%d ready=%d released=%d rebased=%d biasmove=%d "
           "p=%.7g %.7g %.7g v=%.7g q=%.7g ts=%.7g\n",
           tag, (int)g_state.bufferedDeltaCount, (int)g_state.horizonReady,
           (int)g_state.fresh, (int)g_state.rebased,
           (int)g_state.biasMoveExceeded,
           g_state.positionWorldEnu_m[0], g_state.positionWorldEnu_m[1],
           g_state.positionWorldEnu_m[2], g_state.velocityWorldEnu_m_s[0],
           g_state.quaternionWorldBody[0], g_state.timestamp_s);
}

int main(int argc, char **argv) {
    int warm = (argc > 1) ? atoi(argv[1]) : 200;
    int shifted = (argc > 2) ? atoi(argv[2]) : 0;
    memset(&g_state, 0, sizeof(g_state));
    OutputPredictor_startup(&g_state);
    int k;
    for (k = 1; k <= warm; ++k) {
        feed(&g_state, k, 0);
        OutputPredictor_dostep(&g_state);
    }
    report("warm");
    __gcov_reset();
    feed(&g_state, warm + 1, shifted);
    OutputPredictor_dostep(&g_state);
    __gcov_dump();
    report("meas");
    printf("sizeof(State)=%zu sizeof(Scratch_dostep)=%zu sizeof(Scratch_step)=%zu\n",
           sizeof(OutputPredictorState), sizeof(OutputPredictorScratch_dostep),
           sizeof(OutputPredictorScratch_step));
    return 0;
}
