#include <stdlib.h>
#include <string.h>

#include "Tests_VisualNavigationReplay.h"

void *visual_navigation_create(const float *nominal, const float *covariance, int tight)
{
    VisualNavigationReplayState *state = calloc(1, sizeof(*state));
    if (!state) return NULL;
    VisualNavigationReplay_startup(state);
    memcpy(state->previousNominal, nominal, sizeof(state->previousNominal));
    memcpy(state->previousCovariance, covariance, sizeof(state->previousCovariance));
    state->tightCoupling = tight != 0;
    return state;
}

static void copy_input(void *destination, const float **packet, size_t bytes)
{
    memcpy(destination, *packet, bytes);
    *packet += bytes / sizeof(float);
}

void visual_navigation_step(void *session, const float *packet, float *output)
{
    VisualNavigationReplayState *state = session;
    copy_input(state->angularVelocityBody_rad_s, &packet, 3 * sizeof(float));
    copy_input(state->specificForceBody_m_s2, &packet, 3 * sizeof(float));
    state->interval_s = *packet++;
    copy_input(state->imuNoiseDensity, &packet, 2 * sizeof(float));
    state->gpsAvailable = *packet++ != 0;
    state->cameraAvailable = *packet++ != 0;
    copy_input(state->gpsPositionWorld_m, &packet, 3 * sizeof(float));
    copy_input(state->gpsVelocityWorld_m_s, &packet, 3 * sizeof(float));
    copy_input(state->gpsStandardDeviation, &packet, 2 * sizeof(float));
    copy_input(state->landmarksWorld_m, &packet, 12 * sizeof(float));
    copy_input(state->observations, &packet, 12 * sizeof(float));
    copy_input(state->measurementCovariance, &packet, 144 * sizeof(float));
    copy_input(state->rotationBodyCamera, &packet, 9 * sizeof(float));
    copy_input(state->cameraPositionBody_m, &packet, 3 * sizeof(float));
    copy_input(state->intrinsics, &packet, 4 * sizeof(float));
    VisualNavigationReplay_dostep(state);
    memcpy(output, state->nominal, 16 * sizeof(float));
    memcpy(output + 16, state->covariance, 225 * sizeof(float));
    output[241] = state->gpsAccepted;
    output[242] = state->cameraAccepted;
    output[243] = state->gpsNis;
    output[244] = state->cameraNis;
    output[245] = state->rumoca_galec_error_signal_status;
    memcpy(state->previousNominal, state->nominal, sizeof(state->nominal));
    memcpy(state->previousCovariance, state->covariance, sizeof(state->covariance));
}

void visual_navigation_destroy(void *session)
{
    free(session);
}
