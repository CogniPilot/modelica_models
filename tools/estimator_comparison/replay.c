/* SPDX-License-Identifier: Apache-2.0
 * Replay the generated Modelica ESKF with its generated 800 Hz FOH integrator.
 * Native comparator sources are kept in the separately licensed harness.
 */
#ifdef COMPARE_UKF
#include "Estimation_StrapdownINS_UKF_Estimator.h"
typedef EstimatorState NavigationEstimatorState;
#define NavigationEstimator_startup Estimator_startup
#define NavigationEstimator_dostep Estimator_dostep
#define status_gpsPositionCorrectionAccepted gpsPositionCorrectionAccepted
#define status_predictionAccepted predictionAccepted
#else
#include "Vehicles_Rdd2_NavigationEstimator.h"
#endif
#include "Tests_PreintegrationReplay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static NavigationEstimatorState S;
static PreintegrationReplayState I;
static void copy3(float out[3], const float in[3]) { memcpy(out,in,3*sizeof(float)); }
static void copy33(float out[3][3], const float in[3][3]) { memcpy(out,in,9*sizeof(float)); }
int main(int argc,char **argv) {
    if (argc!=5) { fprintf(stderr,"usage: replay IMU.csv MODEL_INPUT.csv OUTPUT.csv gps|denied|transition\n"); return 2; }
    FILE *imu=fopen(argv[1],"r"), *sensors=fopen(argv[2],"r"), *out=fopen(argv[3],"w");
    if (!imu || !sensors || !out) { perror("replay files"); return 1; }
    NavigationEstimator_startup(&S); PreintegrationReplay_startup(&I);
    S.initialTerrainAltitudeWorldEnu_m=-1.0f;
    S.opticalFlowGroundPlaneOffset_m=-1.0f;
#ifndef COMPARE_UKF
    S.pseudoPositionVariance_m2=0; S.zeroVelocityVariance_m2_s2=0;
#else
    // Match RDD2 process noise, retaining UKF's declared initial variances.
    // Matching RDD2's 1e-6 gyro-bias variance causes lowerCholesky to reject
    // the initial prior in float32 (global threshold ~1.8e-6).
    for (int j=0;j<3;j++) {
        S.gyroscope_rad2_s[j][j]=1e-4f;
        S.accelerometer_m2_s3[j][j]=3e-2f;
        S.gyroscopeBias_rad2_s3[j][j]=1e-10f;
        S.accelerometerBias_m2_s5[j][j]=1e-6f;
    }
#endif
    const float field[3]={-1.59e-6f,20.04e-6f,-47.91e-6f};
    copy3(S.localMagneticFieldWorldEnu_T,field);
    for (int i=0;i<3;i++) {
        S.covarianceBody_T2[i][i]=.3e-6f*.3e-6f;
        S.gps_positionCovarianceWorld_m2[i][i]=i==2?.35f*.35f:.2f*.2f;
        S.velocityCovarianceWorld_m2_s2[i][i]=.05f*.05f;
        S.integratedGyroscopeCovariance_rad2[i][i]=1e-10f;
    }
    S.variance_m2=.01f; S.groundDistanceVariance_m2=.0004f;
    S.opticalFlow_integrationTime_s=.01f; S.quality=1;
    for (int i=0;i<2;i++) S.integratedLineOfSightCovariance_rad2[i][i]=1e-8f;
    char line[2048]; fgets(line,sizeof line,imu); fgets(line,sizeof line,sensors);
    fprintf(out,"t_s,e_m,n_m,u_m,ve_m_s,vn_m_s,vu_m_s,qw,qx,qy,qz,pos_valid,att_valid,gps_fused,flow_fused,mag_fused,baro_fused,step_status,prediction_accepted\n");
    unsigned k=0;
    while (fgets(line,sizeof line,imu)) {
        double t; float g[3],a[3];
        if (sscanf(line,"%lf,%f,%f,%f,%f,%f,%f",&t,g,g+1,g+2,a,a+1,a+2)!=7) return 1;
        copy3(I.rate,g); copy3(I.force,a);
        if (k==0) {
            // Seed previous endpoint without integrating a fictional interval.
            copy3(I.previous_previousRate,g); copy3(I.previous_previousForce,a);
            copy3(I.previousRate,g); copy3(I.previousForce,a);
        } else {
            I.clear=(k%8==1);
            if (I.clear) { copy3(I.gyroBias,S.gyroscopeBiasBodyFlu_rad_s); copy3(I.accelBias,S.accelerometerBiasBodyFlu_m_s2); }
            PreintegrationReplay_dostep(&I);
            if (I.rumoca_galec_error_signal_status) { fprintf(stderr,"preintegrator status=%u at %.3f\n",I.rumoca_galec_error_signal_status,t); return 1; }
        }
        if (k%8==0) {
            if (!fgets(line,sizeof line,sensors)) return 1;
            double row[25]; char *q=line,*end;
            for (int j=0;j<25;j++) { row[j]=strtod(q,&end); if (end==q) return 1; q=end+(*end==','); }
            if (fabs(row[0]-t)>1e-7) { fprintf(stderr,"input timestamps differ\n"); return 1; }
            const int gps=strcmp(argv[4],"denied") && !(strcmp(argv[4],"transition")==0 && t>=25 && t<40);
            S.gps_valid=S.positionValid=S.velocityValid=gps;
            S.gps_fresh=gps && row[7]>.5; S.gps_timestamp_s=row[8];
            for (int j=0;j<3;j++) { S.gps_positionWorldEnu_m[j]=row[9+j]; S.gps_velocityWorldEnu_m_s[j]=row[12+j]; }
            S.imu_valid=S.imu_fresh=true; S.imu_timestamp_s=t; S.imu_integrationTime_s=.01f;
            copy3(S.imu_angularVelocityBodyFlu_rad_s,g); copy3(S.specificForceBodyFlu_m_s2,a);
            copy3(S.deltaPositionBodyFlu_m,I.position); copy3(S.deltaVelocityBodyFlu_m_s,I.velocity);
            memcpy(S.deltaQuaternionBodyFlu,I.quaternion,4*sizeof(float));
            copy3(S.gyroscopeBiasLinearizationBodyFlu_rad_s,I.gyroBias); copy3(S.accelerometerBiasLinearizationBodyFlu_m_s2,I.accelBias);
            for (int j=0;j<3;j++) S.deltaAngleBodyFlu_rad[j]=row[1+j]*.01f;
            copy33(S.deltaRotationGyroscopeBiasJacobian_s,I.rotationGyro);
            copy33(S.deltaVelocityGyroscopeBiasJacobian_m,I.velocityGyro);
            copy33(S.deltaVelocityAccelerometerBiasJacobian_s,I.velocityAccel);
            copy33(S.deltaPositionGyroscopeBiasJacobian_m_s,I.positionGyro);
            copy33(S.deltaPositionAccelerometerBiasJacobian_s2,I.positionAccel);
            if (k==0) {
                S.deltaQuaternionBodyFlu[0]=1;
                S.deltaVelocityBodyFlu_m_s[2]=9.81f*.01f;
            }
            S.opticalFlow_valid=S.opticalFlow_fresh=true; S.opticalFlow_timestamp_s=row[15];
            S.groundDistance_m=row[18];
            for (int j=0;j<3;j++) S.integratedGyroscopeBodyFlu_rad[j]=row[1+j]*.01f;
            S.integratedLineOfSight_rad[0]=-row[17]/row[18]*.01f-S.integratedGyroscopeBodyFlu_rad[0];
            S.integratedLineOfSight_rad[1]= row[16]/row[18]*.01f-S.integratedGyroscopeBodyFlu_rad[1];
            S.magnetometer_valid=S.barometer_valid=true;
            S.magnetometer_fresh=S.barometer_fresh=row[19]>.5;
            S.magnetometer_timestamp_s=S.barometer_timestamp_s=row[20];
            for (int j=0;j<3;j++) S.magneticFieldBodyFlu_T[j]=row[21+j];
            S.altitudeWorldEnu_m=row[24];
            NavigationEstimator_dostep(&S);
            fprintf(out,"%.6f",t);
            for(int j=0;j<3;j++) fprintf(out,",%.9g",S.estimate_positionWorldEnu_m[j]);
            for(int j=0;j<3;j++) fprintf(out,",%.9g",S.estimate_velocityWorldEnu_m_s[j]);
            for(int j=0;j<4;j++) fprintf(out,",%.9g",S.estimate_quaternionWorldBody[j]);
            fprintf(out,",%d,%d,%d,%d,%d,%d,%u,%d\n",S.estimate_valid,S.estimate_valid,S.status_gpsPositionCorrectionAccepted,S.status_opticalFlowCorrectionAccepted,S.status_magnetometerCorrectionAccepted,S.status_barometerCorrectionAccepted,S.rumoca_galec_error_signal_status,S.status_predictionAccepted);
        }
        k++;
    }
    fclose(out); fclose(imu); fclose(sensors); return 0;
}
