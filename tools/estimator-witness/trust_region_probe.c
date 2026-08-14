/* Diagnostic: what exactly happens at the first post-ramp acceptance? */
#include "Vehicles_Rdd2_NavigationEstimator.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
void NavigationEstimator_startup(NavigationEstimatorState *self);
void NavigationEstimator_recalibrate(NavigationEstimatorState *self);
void NavigationEstimator_dostep(NavigationEstimatorState *self);
static NavigationEstimatorState s;
static const float DT = 0.001f;
static void sensors(float px, float V) {
    s.mocap_valid = s.mocap_fresh = false;
    s.opticalFlow_valid = s.opticalFlow_fresh = false;
    s.reset = false;
    s.imu_valid = s.imu_fresh = true;
    for (int i = 0; i < 3; i++) { s.imu_angularVelocityBodyFlu_rad_s[i] = 0.0f;
                                  s.specificForceBodyFlu_m_s2[i] = 0.0f; }
    s.specificForceBodyFlu_m_s2[2] = 9.81f;
    s.gps_valid = s.gps_fresh = true; s.positionValid = s.velocityValid = true;
    s.gps_positionWorldEnu_m[0] = px; s.gps_positionWorldEnu_m[1] = 0.0f; s.gps_positionWorldEnu_m[2] = 5.0f;
    s.gps_velocityWorldEnu_m_s[0] = V; s.gps_velocityWorldEnu_m_s[1] = 0.0f; s.gps_velocityWorldEnu_m_s[2] = 0.0f;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        s.gps_positionCovarianceWorld_m2[i][j] = (i == j) ? 0.25f : 0.0f;
        s.velocityCovarianceWorld_m2_s2[i][j]  = (i == j) ? 0.09f : 0.0f;
    }
}
int main(void) {
    float V = 8.0f;
    NavigationEstimator_startup(&s); NavigationEstimator_recalibrate(&s);
    float px = 0.0f;
    sensors(px, V); s.reset = true; NavigationEstimator_dostep(&s);
    int firstAccept = -1;
    printf("tick  stage acc out | est_p=(x,z)          est_v=(x,y,z)                   P_pp    P_vv    quat_y\n");
    for (int i = 0; i < 4000; i++) {
        px += V * DT; sensors(px, V);
        NavigationEstimator_dostep(&s);
        int acc = s.status_gpsPositionCorrectionAccepted;
        if (acc && firstAccept < 0) firstAccept = i;
        int near = (firstAccept >= 0 && i <= firstAccept + 12) || (i % 500 == 0)
                 || (firstAccept < 0 && i > 900 && i % 100 == 0);
        if (near)
          printf("%5d   %d   %d  %d  | %+9.3f %+8.3f  %+9.3f %+9.3f %+9.3f  %8.4g %8.4g %+7.4f\n",
                 i, s.status_recoveryStage, acc, s.status_correctionOutcome,
                 (double)s.estimate_positionWorldEnu_m[0], (double)s.estimate_positionWorldEnu_m[2],
                 (double)s.estimate_velocityWorldEnu_m_s[0], (double)s.estimate_velocityWorldEnu_m_s[1],
                 (double)s.estimate_velocityWorldEnu_m_s[2],
                 (double)s.stateCovariance[0][0], (double)s.stateCovariance[3][3],
                 (double)s.estimate_quaternionWorldBody[2]);
    }
    printf("truth px=%.3f V=%.1f  first accept tick=%d\n", (double)px, (double)V, firstAccept);
    return 0;
}
