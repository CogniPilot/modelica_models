/* KNOWN LIMITATION, pinned so it cannot change silently.
 *
 * A GPS lying by 2 km at 1.67 Hz, with healthy 50 Hz optical flow also
 * present, does NOT fire the recovery ladder. The anchor genuinely
 * alternates between two live sources, so the rejection clock is cleared by
 * a real change of anchor rather than starved by a timeout.
 *
 * This is declared and accepted rather than fixed, on the reasoning that the
 * failure the ladder exists to catch is not present here: the navigation
 * solution stays good because healthy flow anchors it, and the lying GPS is
 * still isolated -- by its own per-source rejection counter, which is the
 * signal added for exactly this job. Detection without false recovery.
 * Making the ladder fire here needs cadence-adaptive staleness (per-source
 * inter-sample interval tracking), a new design surface.
 *
 * What this file asserts is the CURRENT contract, not an aspiration. If any
 * of the three properties below changes -- solution stops being good, the
 * liar stops being visible, or the ladder starts firing -- this test fails by
 * name rather than the behaviour drifting unnoticed.
 *
 * Build: cc -O2 -DAFTER=1 -I$P this.c $P/*.c -lm -o klim && ./klim
 */
#include "Vehicles_Rdd2_NavigationEstimator.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
void NavigationEstimator_startup(NavigationEstimatorState *self);
void NavigationEstimator_dostep(NavigationEstimatorState *self);
static NavigationEstimatorState S;

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
    s->innovationGate = 6.0f; s->samplePeriod = 0.001f;
    s->covarianceInflateWindow_s = 1.0f;
    s->covarianceInflateTimeConstant_s = 0.5f;
    s->aidingDivergentWindow_s = 5.0f;
    s->aidingStaleTimeout_s = 0.5f;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        s->gps_positionCovarianceWorld_m2[i][j] = (i == j) ? 0.25f : 0.0f;
        s->velocityCovarianceWorld_m2_s2[i][j]  = (i == j) ? 0.01f : 0.0f;
    }
    for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
        s->velocityCovarianceBody_m2_s2[i][j] = (i == j) ? 0.01f : 0.0f;
}
static void clr(void) {
    S.gps_valid = S.gps_fresh = false; S.positionValid = S.velocityValid = false;
    S.mocap_valid = S.mocap_fresh = false;
    S.opticalFlow_valid = S.opticalFlow_fresh = false;
}
int main(void) {
    memset(&S, 0, sizeof S); params(&S);
    NavigationEstimator_startup(&S); params(&S);
    S.imu_valid = S.imu_fresh = true; S.specificForceBodyFlu_m_s2[2] = 9.81f;
    double tx = 0.0;
    const int per = (int)(1000.0 / 1.667);
    /* settle on truthful GPS */
    S.gps_valid = S.gps_fresh = true; S.positionValid = S.velocityValid = true;
    NavigationEstimator_dostep(&S);
    for (int t = 0; t < 3000; t++) {
        clr(); tx += 3.0 * 0.001;
        S.gps_valid = S.positionValid = S.velocityValid = true;
        if (t % 100 == 0) { S.gps_fresh = true;
            S.gps_positionWorldEnu_m[0] = (float)tx;
            S.gps_velocityWorldEnu_m_s[0] = 3.0f; }
        NavigationEstimator_dostep(&S);
    }
    /* GPS starts lying by 2 km at 1.67 Hz; healthy 50 Hz flow also present */
    int st1 = 0, st2 = 0, maxGpsRej = 0;
    for (int t = 1; t <= 30000; t++) {
        clr(); tx += 3.0 * 0.001;
        S.gps_valid = S.positionValid = S.velocityValid = true;
        if (t % per == 0) { S.gps_fresh = true;
            S.gps_positionWorldEnu_m[0] = 2000.0f;
            S.gps_positionWorldEnu_m[1] = 2000.0f;
            S.gps_velocityWorldEnu_m_s[0] = 3.0f; }
        S.opticalFlow_valid = true; S.opticalFlow_fresh = (t % 20 == 0);
        S.velocityBodyFlu_m_s[0] = 3.0f;
        S.groundDistance_m = 2.0f; S.quality = 1.0f;
        NavigationEstimator_dostep(&S);
        if (S.status_recoveryStage == 1) st1++;
        if (S.status_recoveryStage == 2) st2++;
        if (S.gpsConsecutiveRejections > maxGpsRej)
            maxGpsRej = S.gpsConsecutiveRejections;
    }
    double errX = (double)S.estimate_positionWorldEnu_m[0] - tx;
    int solutionGood = fabs(errX) < 1.0;
    int liarVisible  = maxGpsRej >= 10;
    int ladderQuiet  = (st1 == 0 && st2 == 0);
    printf("KNOWN LIMITATION: lying GPS @1.67 Hz + healthy 50 Hz flow\n");
    printf("  solution good      errX=%+8.3f m            -> %s\n", errX,
           solutionGood ? "yes" : "NO");
    printf("  liar isolated      gpsConsecutiveRejections peak=%-5d -> %s\n",
           maxGpsRej, liarVisible ? "yes" : "NO");
    printf("  ladder quiet       stage1=%d stage2=%d           -> %s\n",
           st1, st2, ladderQuiet ? "yes" : "NO");
    printf("  estimate.valid=%d\n", S.estimate_valid);
    if (solutionGood && liarVisible && ladderQuiet) {
        printf("\nContract HELD: detection without false recovery.\n");
        return 0;
    }
    printf("\nCONTRACT CHANGED -- this limitation was declared and pinned; "
           "re-evaluate it deliberately:%s%s%s\n",
           solutionGood ? "" : " [solution no longer good]",
           liarVisible  ? "" : " [liar no longer isolated]",
           ladderQuiet  ? "" : " [ladder now fires]");
    return 1;
}
