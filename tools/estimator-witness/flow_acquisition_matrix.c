/* Flow-only truthful acquisition matrix: 4/5/6/8 m/s x 10/50/200/1000 Hz.
 * Optical flow is the ONLY aiding source and it reports the TRUE body
 * velocity. Attitude is not directly observed, so this is the case where a
 * correction direction is valid only at full step. Reports the attitude
 * ratchet explicitly. Build -DAFTER=1 (has recoveryStage) or -DAFTER=0. */
#include "Vehicles_Rdd2_NavigationEstimator.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
void NavigationEstimator_startup(NavigationEstimatorState *self);
void NavigationEstimator_dostep(NavigationEstimatorState *self);
static NavigationEstimatorState S;
static const float DT = 0.001f;

static void params(NavigationEstimatorState *s) {
    s->gravityWorldEnu_m_s2[2] = -9.81f;
    for (int i = 0; i < 3; i++) {
        s->initialVariances_position_m2[i] = 1.0f;
        s->initialVariances_velocity_m2_s2[i] = 1.0f;
        s->initialVariances_attitude_rad2[i] = 0.25f;
        s->initialVariances_gyroscopeBias_rad2_s2[i] = 1.0e-4f;
        s->initialVariances_accelerometerBias_m2_s4[i] = 1.0e-2f;
        s->varianceLimits_position_m2[i] = 1.0e4f;
        s->varianceLimits_velocity_m2_s2[i] = 4.0e2f;
        s->varianceLimits_attitude_rad2[i] = 10.0f;
        s->varianceLimits_gyroscopeBias_rad2_s2[i] = 1.0e-2f;
        s->varianceLimits_accelerometerBias_m2_s4[i] = 1.0f;
        for (int j = 0; j < 3; j++) {
            s->gyroscope_rad2_s[i][j]        = (i == j) ? 1.0e-5f : 0.0f;
            s->accelerometer_m2_s3[i][j]     = (i == j) ? 1.0e-3f : 0.0f;
            s->gyroscopeBias_rad2_s3[i][j]   = (i == j) ? 1.0e-8f : 0.0f;
            s->accelerometerBias_m2_s5[i][j] = (i == j) ? 1.0e-6f : 0.0f;
        }
    }
    s->initialQuaternionWorldBody[0] = 1.0f;
    s->innovationGate = 6.0f;
    s->samplePeriod = DT;
#if AFTER
    s->covarianceInflateWindow_s = 1.0f;
    s->covarianceInflateTimeConstant_s = 0.5f;
    s->aidingDivergentWindow_s = 5.0f;
    s->aidingStaleTimeout_s = 0.5f;
#else
    s->rejectedCorrectionLimit = 50;
#endif
    for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
        s->velocityCovarianceBody_m2_s2[i][j] = (i == j) ? 0.04f : 0.0f;
}
static double quat_angle(void) {   /* total rotation from identity */
    double w = fabs((double)S.estimate_quaternionWorldBody[0]);
    if (w > 1.0) w = 1.0;
    return 2.0 * acos(w);
}
static int failures = 0;
static char failList[512] = "";
static void run(float V, int hz) {
    memset(&S, 0, sizeof S); params(&S);
    NavigationEstimator_startup(&S); params(&S);
    int period = (int)(1000 / hz);
    double truth = 0.0, maxAtt = 0.0;
    int acc = 0, rej = 0;
    for (int k = 0; k < 10000; k++) {           /* 10 s */
        truth += (double)V * DT;
        S.mocap_valid = S.mocap_fresh = false;
        S.gps_valid = S.gps_fresh = false;
        S.positionValid = S.velocityValid = false;
        S.imu_valid = S.imu_fresh = true;
        for (int i = 0; i < 3; i++) { S.imu_angularVelocityBodyFlu_rad_s[i] = 0.0f;
                                      S.specificForceBodyFlu_m_s2[i] = 0.0f; }
        S.specificForceBodyFlu_m_s2[2] = 9.81f;
        S.opticalFlow_valid = true;
        S.opticalFlow_fresh = (k % period == 0);
        S.velocityBodyFlu_m_s[0] = V;            /* TRUTHFUL */
        S.velocityBodyFlu_m_s[1] = 0.0f;
        S.integrationTime_s = (float)period * DT;
        S.groundDistance_m = 3.0f; S.quality = 1.0f;
        NavigationEstimator_dostep(&S);
        if (S.opticalFlow_fresh) { if (S.status_opticalFlowCorrectionAccepted) acc++; else rej++; }
        double a = quat_angle(); if (a > maxAtt) maxAtt = a;
    }
    double errX = (double)S.estimate_positionWorldEnu_m[0] - truth;
#if AFTER
    int stage = S.status_recoveryStage;
#else
    int stage = -1;
#endif
    int ok = fabs((double)S.estimate_velocityWorldEnu_m_s[0] - (double)V) < 0.5;
    if (!ok) { failures++; snprintf(failList + strlen(failList),
        sizeof failList - strlen(failList), " V=%.0f/%dHz", (double)V, hz); }
    printf("  V=%4.1f %5dHz : est_v=%+9.3f (truth %4.1f) errX=%+11.3f maxAtt=%7.4f rad "
           "flowAcc=%4d flowRej=%4d stage=%d valid=%d  %s\n",
           (double)V, hz, (double)S.estimate_velocityWorldEnu_m_s[0], (double)V,
           errX, maxAtt, acc, rej, stage, S.estimate_valid,
           ok ? "tracks" : "*** DIVERGES ***");
}
int main(void) {
    printf("######## FLOW-ONLY TRUTHFUL ACQUISITION (%s) ########\n",
           AFTER ? "hardened" : "base");
    const float Vs[] = {4.0f, 5.0f, 6.0f, 8.0f};
    const int Hz[] = {10, 50, 200, 1000};
    for (int v = 0; v < 4; v++) { for (int h = 0; h < 4; h++) run(Vs[v], Hz[h]); }
    /* Name every failing cell. A witness whose failure mode is a hang or a
     * bare non-zero status tells the next reader nothing about WHICH
     * speed-rate combination regressed. */
    if (failures) {
        printf("\nFAILED %d of 16 cells:%s\n", failures, failList);
        return 1;
    }
    printf("\nAll 16 speed x rate cells track.\n");
    return 0;
}
