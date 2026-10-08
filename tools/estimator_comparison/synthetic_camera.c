#include <string.h>

#include "Tests_SyntheticLandmarkReplay.h"

void synthetic_camera_project(const float *pose, const float *landmarks,
    const float *rotation, const float *lever, const float *intrinsics,
    const float *error, float *output)
{
    SyntheticLandmarkReplayState state;
    SyntheticLandmarkReplay_startup(&state);
    memcpy(state.positionWorld_m, pose, 3 * sizeof(float));
    memcpy(state.quaternionWorldBody, pose + 3, 4 * sizeof(float));
    memcpy(state.landmarksWorld_m, landmarks, 12 * sizeof(float));
    memcpy(state.rotationBodyCamera, rotation, 9 * sizeof(float));
    memcpy(state.cameraPositionBody_m, lever, 3 * sizeof(float));
    memcpy(state.intrinsics, intrinsics, 4 * sizeof(float));
    memcpy(state.measurementError, error, 12 * sizeof(float));
    state.imageSize_pixels[0] = 640;
    state.imageSize_pixels[1] = 480;
    state.depthRange_m[0] = 0.1f;
    state.depthRange_m[1] = 10.0f;
    state.available = 1;
    for (int landmark = 0; landmark < 4; ++landmark) state.detected[landmark] = 1;
    SyntheticLandmarkReplay_dostep(&state);
    memcpy(output, state.observations, 12 * sizeof(float));
    for (int landmark = 0; landmark < 4; ++landmark) output[12 + landmark] = state.visible[landmark];
    output[16] = state.rumoca_galec_error_signal_status;
}
