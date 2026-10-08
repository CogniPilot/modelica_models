/* SPDX-License-Identifier: Apache-2.0 */
#include "Tests_PreintegrationReplay.h"
#include "Tests_PreintegrationNoiseReplay.h"
#include <string.h>

int integrate_packet(int first_order, int intervals, const float *rates,
                     const float *forces, float *output) {
    PreintegrationReplayState integrator = {0};
    PreintegrationReplay_startup(&integrator);
    integrator.useFirstOrderHold = first_order;
    memcpy(integrator.previous_previousRate, rates, 3 * sizeof(float));
    memcpy(integrator.previous_previousForce, forces, 3 * sizeof(float));
    memcpy(integrator.previousRate, rates, 3 * sizeof(float));
    memcpy(integrator.previousForce, forces, 3 * sizeof(float));
    for (int sample = 1; sample <= intervals; ++sample) {
        memcpy(integrator.rate, rates + 3 * sample, 3 * sizeof(float));
        memcpy(integrator.force, forces + 3 * sample, 3 * sizeof(float));
        integrator.clear = sample == 1;
        PreintegrationReplay_dostep(&integrator);
        if (integrator.rumoca_galec_error_signal_status) return 1;
    }
    memcpy(output, integrator.position, 3 * sizeof(float));
    memcpy(output + 3, integrator.velocity, 3 * sizeof(float));
    memcpy(output + 6, integrator.quaternion, 4 * sizeof(float));
    return 0;
}

int packet_covariance(const float *rate, const float *force, float interval,
                      float gyro_density, float accel_density, float *output) {
    PreintegrationNoiseReplayState noise = {0};
    PreintegrationNoiseReplay_startup(&noise);
    memcpy(noise.angularVelocity, rate, 3 * sizeof(float));
    memcpy(noise.specificForce, force, 3 * sizeof(float));
    noise.interval_s = interval;
    noise.gyroscopeDensity = gyro_density;
    noise.accelerometerDensity = accel_density;
    PreintegrationNoiseReplay_dostep(&noise);
    memcpy(output, noise.covariance, 225 * sizeof(float));
    return noise.rumoca_galec_error_signal_status ? 1 : 0;
}
