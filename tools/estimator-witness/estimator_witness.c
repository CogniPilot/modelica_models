/* v2 witness: NaN family, outlier flight, ATTITUDE GUARANTEE through
 * stages 1 and 2, and the nominal %a trace. Builds against base (-DAFTER=0)
 * and hardened (-DAFTER=1). */
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
}
static void cov(NavigationEstimatorState *s) {
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        s->gps_positionCovarianceWorld_m2[i][j] = (i == j) ? 0.25f : 0.0f;
        s->velocityCovarianceWorld_m2_s2[i][j]  = (i == j) ? 0.09f : 0.0f;
        s->mocap_positionCovarianceWorld_m2[i][j] = (i == j) ? 1.0e-4f : 0.0f;
        s->attitudeCovarianceBody_rad2[i][j] = (i == j) ? 1.0e-4f : 0.0f;
    }
    for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
        s->velocityCovarianceBody_m2_s2[i][j] = (i == j) ? 0.04f : 0.0f;
}
static void boot(void) {
    memset(&S, 0, sizeof S); params(&S);
    NavigationEstimator_startup(&S); params(&S); cov(&S);
    S.imu_valid = true; S.imu_fresh = true;
    S.specificForceBodyFlu_m_s2[2] = 9.81f;
}
static void gps(int on, float e, float n, float u) {
    S.gps_valid = on; S.gps_fresh = on;
    S.positionValid = on; S.velocityValid = on;
    S.gps_positionWorldEnu_m[0] = e; S.gps_positionWorldEnu_m[1] = n;
    S.gps_positionWorldEnu_m[2] = u;
}
static double yaw_of_q(void) {
    double w=S.estimate_quaternionWorldBody[0], x=S.estimate_quaternionWorldBody[1];
    double y=S.estimate_quaternionWorldBody[2], z=S.estimate_quaternionWorldBody[3];
    return atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
}
/* total rotation angle of the published quaternion away from a reference */
static double quat_angle_from(const double r[4]) {
    double d = r[0]*S.estimate_quaternionWorldBody[0] + r[1]*S.estimate_quaternionWorldBody[1]
             + r[2]*S.estimate_quaternionWorldBody[2] + r[3]*S.estimate_quaternionWorldBody[3];
    d = fabs(d); if (d > 1.0) d = 1.0;
    return 2.0 * acos(d);
}
static const char *fin(float v){return isnan((double)v)?"NaN":isinf((double)v)?"Inf":"finite";}
static int state_finite(void){
    for(int i=0;i<3;i++) if(!isfinite((double)S.estimate_positionWorldEnu_m[i])
        || !isfinite((double)S.estimate_velocityWorldEnu_m_s[i])) return 0;
    for(int i=0;i<4;i++) if(!isfinite((double)S.estimate_quaternionWorldBody[i])) return 0;
    return 1;
}

static void scenario_nan(void) {
    struct { const char *n; int w; float v; } bad[] = {
        {"gps pos E = NaN",0,(float)NAN},{"gps pos E = +Inf",0,(float)INFINITY},
        {"gps vel E = NaN",1,(float)NAN},{"gps vel E = +Inf",1,(float)INFINITY},
        {"gps posCov NaN",2,(float)NAN},{"gps posCov 0",2,0.0f},{"gps posCov -1",2,-1.0f},
        {"mocap pos NaN",4,(float)NAN},{"flow vel NaN",5,(float)NAN},{"flow cov NaN",6,(float)NAN},
        {"imu accel NaN",7,(float)NAN},{"imu gyro NaN",8,(float)NAN},
    };
    printf("\n===== A. NON-FINITE INPUTS, PER CHANNEL =====\n");
    printf("%-18s %-4s %-5s %-8s %-6s %-9s %s\n","channel","acc","out","estPos","valid","after500","verdict");
    for (unsigned c=0;c<sizeof bad/sizeof bad[0];c++){
        boot(); gps(1,0,0,0); NavigationEstimator_dostep(&S);
        gps(0,0,0,0); for(int i=0;i<20;i++) NavigationEstimator_dostep(&S);
        gps(1,1.0f,0,0);
        int flow=0,moc=0;
        switch(bad[c].w){
        case 0: S.gps_positionWorldEnu_m[0]=bad[c].v; break;
        case 1: S.gps_velocityWorldEnu_m_s[0]=bad[c].v; break;
        case 2: for(int i=0;i<3;i++) S.gps_positionCovarianceWorld_m2[i][i]=bad[c].v; break;
        case 4: gps(0,0,0,0); moc=1; S.mocap_valid=S.mocap_fresh=true;
                S.mocap_quaternionWorldBody[0]=1.0f; S.mocap_positionWorldEnu_m[0]=bad[c].v; break;
        case 5: gps(0,0,0,0); flow=1; S.opticalFlow_valid=S.opticalFlow_fresh=true;
                S.velocityBodyFlu_m_s[0]=bad[c].v; break;
        case 6: gps(0,0,0,0); flow=1; S.opticalFlow_valid=S.opticalFlow_fresh=true;
                for(int i=0;i<2;i++) S.velocityCovarianceBody_m2_s2[i][i]=bad[c].v; break;
        case 7: S.specificForceBodyFlu_m_s2[0]=bad[c].v; break;
        case 8: S.imu_angularVelocityBodyFlu_rad_s[0]=bad[c].v; break;
        }
        NavigationEstimator_dostep(&S);
        int acc = flow?S.status_opticalFlowCorrectionAccepted
                : moc?S.status_mocapCorrectionAccepted:S.status_gpsPositionCorrectionAccepted;
#if AFTER
        int out=S.status_correctionOutcome;
#else
        int out=S.status_innovationGateRejected?3:(acc?1:0);
#endif
        int v=S.estimate_valid;
        boot_again:; cov(&S);
        S.specificForceBodyFlu_m_s2[0]=0.0f; S.imu_angularVelocityBodyFlu_rad_s[0]=0.0f;
        for(int k=0;k<500;k++){ gps(1,1.0f,0,0); S.mocap_valid=false; S.opticalFlow_valid=false;
            NavigationEstimator_dostep(&S);}
        printf("%-18s %-4d %-5d %-8s %-6d %-9s %s\n",bad[c].n,acc,out,
               fin(S.estimate_positionWorldEnu_m[0]),v,fin(S.estimate_positionWorldEnu_m[0]),
               state_finite()?"clean":"*** POISONED ***");
    }
}

/* ATTITUDE GUARANTEE: attitude must stay bounded vs truth through stages 1-2 */
static void scenario_attitude(void) {
    printf("\n===== B. ATTITUDE GUARANTEE THROUGH STAGE 1 AND STAGE 2 =====\n");
    boot(); gps(1,0,0,0); NavigationEstimator_dostep(&S);
    /* build a real yaw of 1.0 rad under good GPS, flying 3 m/s east */
    S.imu_angularVelocityBodyFlu_rad_s[2]=0.5f;
    double px=0.0;
    for(int k=0;k<2000;k++){ px+=3.0*DT; gps(1,(float)px,0,0);
        S.gps_velocityWorldEnu_m_s[0]=3.0f; NavigationEstimator_dostep(&S);}
    S.imu_angularVelocityBodyFlu_rad_s[2]=0.0f;
    double qref[4]={S.estimate_quaternionWorldBody[0],S.estimate_quaternionWorldBody[1],
                    S.estimate_quaternionWorldBody[2],S.estimate_quaternionWorldBody[3]};
    double yaw0=yaw_of_q(), vE0=S.estimate_velocityWorldEnu_m_s[0];
    printf("  truth-locked reference: yaw=%+.4f vE=%+.3f pos=%+.2f\n",yaw0,vE0,px);
    /* now a consistent 2 km outlier stream: truth keeps flying straight and level */
    double maxAtt=0.0; int st1=-1,st2=-1,everInvalid=0;
    for(int k=1;k<=9000;k++){
        px+=3.0*DT;
        gps(1,2000.0f,2000.0f,2000.0f); S.gps_velocityWorldEnu_m_s[0]=3.0f;
        NavigationEstimator_dostep(&S);
        double a=quat_angle_from(qref); if(a>maxAtt) maxAtt=a;
        if(!S.estimate_valid) everInvalid++;
#if AFTER
        if(S.status_recoveryStage>=1&&st1<0) st1=k;
        if(S.status_recoveryStage>=2&&st2<0) st2=k;
#else
        if(S.status_covarianceReinitialized&&st1<0) st1=k;
#endif
    }
    printf("  after 9 s of 2 km outliers: stage1@%dms stage2@%dms invalidTicks=%d\n",st1,st2,everInvalid);
    printf("  ATTITUDE drift from truth-locked reference: max=%.5f rad (%.3f deg)  %s\n",
           maxAtt,maxAtt*180.0/M_PI, maxAtt<0.05?"BOUNDED":"*** ATTITUDE CORRUPTED ***");
    printf("  yaw %+.4f -> %+.4f | vE %+.3f -> %+.3f | posE %+.2f (truth %+.2f)\n",
           yaw0,yaw_of_q(),vE0,(double)S.estimate_velocityWorldEnu_m_s[0],
           (double)S.estimate_positionWorldEnu_m[0],px);
    printf("  teleported to outlier? %s | estimate.valid=%d\n",
           fabs((double)S.estimate_positionWorldEnu_m[0]-2000.0)<100.0?"*** YES ***":"no",
           S.estimate_valid);
    /* recovery on truthful GPS */
    for(int k=0;k<4000;k++){ px+=3.0*DT; gps(1,(float)px,0,0);
        S.gps_velocityWorldEnu_m_s[0]=3.0f; NavigationEstimator_dostep(&S);}
    printf("  after 4 s truthful GPS: posE=%+.3f (truth %+.3f) vE=%+.3f attErr=%.5f rad valid=%d %s\n",
           (double)S.estimate_positionWorldEnu_m[0],px,(double)S.estimate_velocityWorldEnu_m_s[0],
           quat_angle_from(qref),S.estimate_valid,
           fabs((double)S.estimate_positionWorldEnu_m[0]-px)<2.0?"RECOVERED":"*** NOT RECOVERED ***");
}

static void scenario_nominal(const char *path) {
    FILE *f=fopen(path,"w");
    boot();
    /* Genuinely nominal: truth starts at rest exactly where the filter
     * initializes, then accelerates gently at 0.5 m/s2. Every correction
     * stays small, nothing is ever gate-rejected, and no recovery stage is
     * ever entered -- so this is the happy path the bit-identity claim is
     * about. A cold start against an already-moving stream is NOT nominal:
     * it is a large-innovation acquisition, which is exactly what this
     * change alters on purpose. */
    const double ACC=0.5;
    for(int k=0;k<4000;k++){
        double t=k*(double)DT;
        double vx=ACC*t, px=0.5*ACC*t*t;
        gps(1,(float)px,0.0f,0.0f);
        S.gps_fresh=(k%100==0);
        S.gps_velocityWorldEnu_m_s[0]=(float)vx; S.gps_velocityWorldEnu_m_s[1]=0.0f;
        S.specificForceBodyFlu_m_s2[0]=(float)ACC;
        S.opticalFlow_valid=true; S.opticalFlow_fresh=(k%20==0);
        S.velocityBodyFlu_m_s[0]=(float)vx; S.velocityBodyFlu_m_s[1]=0.0f;
        S.integrationTime_s=0.02f; S.groundDistance_m=3.0f; S.quality=1.0f;
        NavigationEstimator_dostep(&S);
        fprintf(f,"%d",k);
        for(int i=0;i<3;i++) fprintf(f," %a",(double)S.estimate_positionWorldEnu_m[i]);
        for(int i=0;i<3;i++) fprintf(f," %a",(double)S.estimate_velocityWorldEnu_m_s[i]);
        for(int i=0;i<4;i++) fprintf(f," %a",(double)S.estimate_quaternionWorldBody[i]);
        for(int i=0;i<15;i++) fprintf(f," %a",(double)S.stateCovariance[i][i]);
        fprintf(f," %d %d\n",S.estimate_valid,S.status_gpsPositionCorrectionAccepted);
    }
    fclose(f);
    printf("\n===== C. NOMINAL 4000-TICK TRACE -> %s =====\n  final pos=(%+.4f,%+.4f,%+.4f) valid=%d\n",
        path,(double)S.estimate_positionWorldEnu_m[0],(double)S.estimate_positionWorldEnu_m[1],
        (double)S.estimate_positionWorldEnu_m[2],S.estimate_valid);
}

static void scenario_reset(void){
    printf("\n===== D. COMMANDED RESET =====\n");
    boot(); gps(1,0,0,0); NavigationEstimator_dostep(&S);
    for(int i=0;i<50;i++){gps(1,5.0f,0,0); NavigationEstimator_dostep(&S);}
    printf("  before reset: pos=%+.3f valid=%d\n",(double)S.estimate_positionWorldEnu_m[0],S.estimate_valid);
    S.reset=true; gps(1,100.0f,0,0); NavigationEstimator_dostep(&S); S.reset=false;
    printf("  after reset  : pos=%+.3f P_EE=%.4g valid=%d stage=%d  %s\n",
      (double)S.estimate_positionWorldEnu_m[0],(double)S.stateCovariance[0][0],
      S.estimate_valid,
#if AFTER
      S.status_recoveryStage,
#else
      0,
#endif
      fabs((double)S.estimate_positionWorldEnu_m[0]-100.0)<1.0?"RE-SEEDED from aiding":"*** NOT re-seeded ***");
}

int main(int argc,char**argv){
    printf("############ %s ############\n", AFTER?"HEAD (hardened v2)":"BASE (consolidated 5f2dec5)");
    scenario_nan();
    scenario_attitude();
    scenario_reset();
    scenario_nominal(argc>1?argv[1]:"trace.txt");
    return 0;
}
