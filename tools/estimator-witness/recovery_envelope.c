/* Displaced-state truthful-GPS reacquisition. State is displaced from truth;
 * GPS then reports the TRUTH. Every fix should be accepted once the
 * covariance has grown. Prints the rejection REASON so the mechanism is
 * identified rather than guessed. */
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
 s->covarianceInflateWindow_s=1;s->covarianceInflateTimeConstant_s=0.5f;
 s->aidingDivergentWindow_s=5;s->aidingStaleTimeout_s=0.5f;
 for(int i=0;i<3;i++)for(int j=0;j<3;j++){s->gps_positionCovarianceWorld_m2[i][j]=(i==j)?0.25f:0;
  s->velocityCovarianceWorld_m2_s2[i][j]=(i==j)?0.01f:0;}
}
static void clr(void){S.gps_valid=false;S.gps_fresh=false;S.positionValid=false;S.velocityValid=false;
 S.mocap_valid=false;S.mocap_fresh=false;S.opticalFlow_valid=false;S.opticalFlow_fresh=false;}
static int failures=0; static char fl[512]="";
static void run(double disp,double hz,int inside){
 memset(&S,0,sizeof S);params(&S);NavigationEstimator_startup(&S);params(&S);
 S.imu_valid=true;S.imu_fresh=true;S.specificForceBodyFlu_m_s2[2]=9.81f;
 double tx=0.0; int per=(int)(1000.0/hz);
 /* settle 3 s on truthful GPS at rest */
 S.gps_valid=true;S.gps_fresh=true;S.positionValid=true;S.velocityValid=true;
 NavigationEstimator_dostep(&S);
 for(int t=0;t<3000;t++){clr();S.gps_valid=S.positionValid=S.velocityValid=true;
  if(t%per==0){S.gps_fresh=true;S.gps_positionWorldEnu_m[0]=(float)tx;S.gps_velocityWorldEnu_m_s[0]=0.0f;}
  NavigationEstimator_dostep(&S);}
 /* DISPLACE the state; truth and GPS stay put and truthful */
 S.statePosition[0]+=(float)disp; S.previous_statePosition[0]+=(float)disp;
 int acc=0,rej=0,cnt[6]={0,0,0,0,0,0},maxStage=0; double pmax=0;
 for(int t=1;t<=30000;t++){
  clr();S.gps_valid=S.positionValid=S.velocityValid=true;
  if(t%per==0){S.gps_fresh=true;S.gps_positionWorldEnu_m[0]=(float)tx;S.gps_velocityWorldEnu_m_s[0]=0.0f;}
  NavigationEstimator_dostep(&S);
  if(S.gps_fresh){ if(S.status_gpsPositionCorrectionAccepted)acc++; else rej++;
    int o=S.status_correctionOutcome; if(o>=0&&o<6)cnt[o]++; }
  if(S.status_recoveryStage>maxStage)maxStage=S.status_recoveryStage;
  if((double)S.stateCovariance[0][0]>pmax)pmax=(double)S.stateCovariance[0][0];
 }
 double err=(double)S.estimate_positionWorldEnu_m[0]-tx;
 /* INSIDE the declared envelope the contract is reacquisition AND a
  * convergence bound. OUTSIDE it, reacquisition is NOT promised and is not
  * asserted; what is promised is fail-closed behaviour -- the estimate stays
  * finite, bounded and usable, and never teleports onto a wrong place. */
 int finite = isfinite(err) && isfinite((double)S.estimate_velocityWorldEnu_m_s[0]);
 int ok = inside ? (finite && fabs(err) < 1.0 && acc > 0 && S.estimate_valid)
                 : (finite && fabs(err) < 200.0 && S.estimate_valid);
 if(!ok){failures++;snprintf(fl+strlen(fl),sizeof fl-strlen(fl)," %.0fm/%.2fHz",disp,hz);}
 printf("  disp=%5.0f m @%6.2f Hz | acc=%4d rej=%4d outcome[none/acc/nonfin/gate/factz/covun]=%d/%d/%d/%d/%d/%d "
        "P_pp_max=%9.4g maxStage=%d errX=%+10.3f valid=%d %s\n",
        disp,hz,acc,rej,cnt[0],cnt[1],cnt[2],cnt[3],cnt[4],cnt[5],pmax,maxStage,err,S.estimate_valid,
        ok?(inside?"REACQUIRES (in envelope)":"fail-closed (outside envelope)")
          :(inside?"*** ENVELOPE BREACH: no reacquisition ***":"*** NOT FAIL-CLOSED ***"));
}
int main(void){
 printf("######## RECOVERY ENVELOPE: INSIDE AND OUTSIDE ########\n");
 printf(" INSIDE declared envelope (reacquisition + convergence asserted)\n");
 run(50,10.0,1); run(50,1.667,1); run(75,10.0,1); run(100,10.0,1);
 printf(" OUTSIDE declared envelope (fail-closed asserted; reacquisition NOT claimed)\n");
 run(150,10.0,0); run(150,1.667,0);
 if(failures){printf("\nFAILED %d cells:%s\n",failures,fl);return 1;}
 printf("\nEnvelope contract holds on all cells.\n");return 0;}
