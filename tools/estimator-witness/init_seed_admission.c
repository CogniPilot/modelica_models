/* Valid-flagged NaN GPS at reset/init: must NOT poison the state, and the
 * filter must recover fully on subsequent clean fixes. Discriminating. */
#include "Vehicles_Rdd2_NavigationEstimator.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
void NavigationEstimator_startup(NavigationEstimatorState*);
void NavigationEstimator_dostep(NavigationEstimatorState*);
static NavigationEstimatorState S;
static void params(NavigationEstimatorState*s){
 s->gravityWorldEnu_m_s2[2]=-9.81f;
 for(int i=0;i<3;i++){s->initialVariances_position_m2[i]=1;s->initialVariances_velocity_m2_s2[i]=1;
  s->initialVariances_attitude_rad2[i]=0.25f;s->initialVariances_gyroscopeBias_rad2_s2[i]=1e-4f;
  s->initialVariances_accelerometerBias_m2_s4[i]=1e-2f;s->varianceLimits_position_m2[i]=1e4f;
  s->varianceLimits_velocity_m2_s2[i]=4e2f;s->varianceLimits_attitude_rad2[i]=10;
  s->varianceLimits_gyroscopeBias_rad2_s2[i]=1e-2f;s->varianceLimits_accelerometerBias_m2_s4[i]=1;
  for(int j=0;j<3;j++){s->gyroscope_rad2_s[i][j]=(i==j)?1e-5f:0;s->accelerometer_m2_s3[i][j]=(i==j)?1e-3f:0;
   s->gyroscopeBias_rad2_s3[i][j]=(i==j)?1e-8f:0;s->accelerometerBias_m2_s5[i][j]=(i==j)?1e-6f:0;}}
 s->initialQuaternionWorldBody[0]=1;s->innovationGate=6;s->samplePeriod=0.001f;
#if AFTER
 s->covarianceInflateWindow_s=1;s->covarianceInflateTimeConstant_s=0.5f;
 s->aidingDivergentWindow_s=5;s->aidingStaleTimeout_s=0.5f;
#endif
 for(int i=0;i<3;i++)for(int j=0;j<3;j++){s->gps_positionCovarianceWorld_m2[i][j]=(i==j)?0.25f:0;
  s->velocityCovarianceWorld_m2_s2[i][j]=(i==j)?0.01f:0;
  s->mocap_positionCovarianceWorld_m2[i][j]=(i==j)?1e-4f:0;
  s->attitudeCovarianceBody_rad2[i][j]=(i==j)?1e-4f:0;}
}
static int fails=0;
static void one(const char*name,int mode){
 /* mode: 0 gps NaN, 1 mocap pos NaN, 2 mocap zero-norm quat, 3 near-zero-norm */
 memset(&S,0,sizeof S);params(&S);NavigationEstimator_startup(&S);params(&S);
 S.imu_valid=true;S.imu_fresh=true;S.specificForceBodyFlu_m_s2[2]=9.81f;
 /* FIRST tick is the initialization tick: aiding is valid-flagged but NaN */
 if(mode==0){ S.gps_valid=true;S.gps_fresh=true;S.positionValid=true;S.velocityValid=true;
   S.gps_positionWorldEnu_m[0]=(float)NAN; }
 else { S.mocap_valid=true;S.mocap_fresh=true;
   S.mocap_positionWorldEnu_m[0]=(mode==1)?(float)NAN:5.0f;
   /* A finite quaternion that cannot be normalized: every component
    * passes a magnitude check, yet normalize() divides by ~0. */
   float n = (mode==2)?0.0f:((mode==3)?1.0e-7f:1.0f);
   S.mocap_quaternionWorldBody[0]=n;
   S.mocap_quaternionWorldBody[1]=0.0f;
   S.mocap_quaternionWorldBody[2]=0.0f;
   S.mocap_quaternionWorldBody[3]=0.0f; }
 NavigationEstimator_dostep(&S);
 int initValid=S.estimate_valid; double p0=(double)S.estimate_positionWorldEnu_m[0];
 /* 500 clean truthful fixes at the origin */
 for(int k=0;k<500;k++){
   S.mocap_valid=S.mocap_fresh=false;
   S.gps_valid=S.gps_fresh=true;S.positionValid=S.velocityValid=true;
   for(int i=0;i<3;i++){S.gps_positionWorldEnu_m[i]=0.0f;S.gps_velocityWorldEnu_m_s[i]=0.0f;}
   NavigationEstimator_dostep(&S);
 }
 double pf=(double)S.estimate_positionWorldEnu_m[0];
 /* quaternion must remain a usable unit rotation, never garbage */
 double qn=0; for(int i=0;i<4;i++) qn+=(double)S.estimate_quaternionWorldBody[i]*S.estimate_quaternionWorldBody[i];
 int qok = isfinite(qn) && fabs(sqrt(qn)-1.0) < 1e-3;
 int ok = isfinite(p0) && isfinite(pf) && fabs(pf)<1.0 && S.estimate_valid && qok;
 if(!ok)fails++;
 printf("  %-22s initPos=%-10s initValid=%d | after 500 clean: pos=%-10s valid=%d  %s\n",
        name, isfinite(p0)?"finite":"NaN", initValid,
        isfinite(pf)?"finite":"NaN", S.estimate_valid,
        ok?"recovers":"*** PERMANENTLY POISONED ***");
}
int main(void){
 printf("######## VALID-FLAGGED NaN AIDING AT RESET/INIT ########\n");
 one("gps pos NaN at init",0);
 one("mocap pos NaN at init",1);
 one("mocap zero-norm quat",2);
 one("mocap near-zero-norm quat",3);
 if(fails){printf("\nFAILED %d of 4 cases\n",fails);return 1;}
 printf("\nAll init-path seed cases admit safely.\n");return 0;}
